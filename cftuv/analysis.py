from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
import math
from types import MappingProxyType

import bmesh
import bpy
from mathutils import Vector

try:
    from .constants import (
        CORNER_ANGLE_THRESHOLD_DEG,
        FLOOR_THRESHOLD,
        FRAME_ALIGNMENT_THRESHOLD_H,
        FRAME_ALIGNMENT_THRESHOLD_V,
        NB_MESH_BORDER,
        NB_SEAM_SELF,
        WALL_THRESHOLD,
        WORLD_UP,
    )
    from .model import (
        BoundaryChain,
        BoundaryCorner,
        BoundaryLoop,
        ChainNeighborKind,
        CornerKind,
        FrameRole,
        LoopKind,
        PatchGraph,
        PatchNode,
        PatchType,
        WorldFacing,
        SeamEdge,
        MeshPreflightIssue,
        MeshPreflightReport,
        FormattedReport,
    )
except ImportError:
    from constants import (
        CORNER_ANGLE_THRESHOLD_DEG,
        FLOOR_THRESHOLD,
        FRAME_ALIGNMENT_THRESHOLD_H,
        FRAME_ALIGNMENT_THRESHOLD_V,
        NB_MESH_BORDER,
        NB_SEAM_SELF,
        WALL_THRESHOLD,
        WORLD_UP,
    )
    from model import (
        BoundaryChain,
        BoundaryCorner,
        BoundaryLoop,
        ChainNeighborKind,
        CornerKind,
        FrameRole,
        LoopKind,
        PatchGraph,
        PatchNode,
        PatchType,
        WorldFacing,
        SeamEdge,
        MeshPreflightIssue,
        MeshPreflightReport,
        FormattedReport,
    )


@dataclass
class _RawBoundaryLoop:
    """Private typed payload for one traced raw boundary loop."""

    vert_indices: list[int] = field(default_factory=list)
    vert_cos: list[Vector] = field(default_factory=list)
    edge_indices: list[int] = field(default_factory=list)
    side_face_indices: list[int] = field(default_factory=list)
    kind: LoopKind = LoopKind.OUTER
    depth: int = 0
    closed: bool = False


@dataclass
class _RawBoundaryChain:
    """Private typed payload for one intermediate raw chain."""

    vert_indices: list[int] = field(default_factory=list)
    vert_cos: list[Vector] = field(default_factory=list)
    edge_indices: list[int] = field(default_factory=list)
    side_face_indices: list[int] = field(default_factory=list)
    neighbor: int = NB_MESH_BORDER
    is_closed: bool = False
    start_loop_index: int = 0
    end_loop_index: int = 0
    is_corner_split: bool = False


@dataclass
class _RawPatchBoundaryData:
    """Private typed payload for one patch before BoundaryLoop serialization."""

    face_indices: list[int] = field(default_factory=list)
    raw_loops: list[_RawBoundaryLoop] = field(default_factory=list)
    basis_u: Vector = field(default_factory=lambda: Vector((1.0, 0.0, 0.0)))
    basis_v: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 1.0)))


@dataclass
class _PatchTopologyAssemblyState:
    """Typed patch assembly contract from raw patch trace to final PatchNode."""

    patch_id: int
    node: PatchNode
    raw_boundary_data: _RawPatchBoundaryData


@dataclass
class _BoundaryLoopBuildState:
    """Private step-by-step build contract for one finalized BoundaryLoop."""

    raw_loop: _RawBoundaryLoop
    boundary_loop: BoundaryLoop
    loop_neighbors: list[int] = field(default_factory=list)
    raw_chains: list[_RawBoundaryChain] = field(default_factory=list)


@dataclass
class _BoundaryLoopDerivedTopology:
    """Final derived topology written back into BoundaryLoop."""

    chains: list[BoundaryChain] = field(default_factory=list)
    corners: list[BoundaryCorner] = field(default_factory=list)
    uses_geometric_corner_fallback: bool = False


@dataclass
class _AnalysisUvClassificationState:
    """Rollback contract for the explicit UV-dependent OUTER/HOLE boundary step."""

    temp_uv_name: str = ""
    original_active_uv_name: str | None = None
    original_selection: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class _CornerTurnCandidate:
    index: int
    turn_angle_deg: float


@dataclass(frozen=True)
class _CornerDetectionPolicy:
    """Shared corner-detection policy for closed and open topology paths."""

    closed_loop: bool
    min_spacing_vertices: int
    min_corner_count: int = 0
    loop_min_span_fraction: float = 0.0
    relaxed_loop_min_span_fraction: float = 0.0
    reject_projected_hairpins: bool = False
    filter_all_bevel_turns: bool = False
    filter_hairpin_bevel_turns: bool = False
    allow_raw_non_hairpin_fallback: bool = False


@dataclass(frozen=True)
class _OpenBorderCornerDetectionResult:
    """Detection stages for one open border chain before topology split."""

    candidate_indices: tuple[int, ...] = ()
    supported_indices: tuple[int, ...] = ()


@dataclass(frozen=True)
class _BoundarySideKey:
    face_index: int
    edge_index: int
    vert_index: int


@dataclass(frozen=True)
class _PlanarPoint2D:
    x: float
    y: float


@dataclass(frozen=True, order=True)
class _PolygonEdgeLengthCandidate:
    len_squared: float
    index: int


@dataclass(frozen=True, order=True)
class _ChainFrameConfidence:
    primary_support: float
    total_length: float
    avg_deviation_score: float
    max_deviation_score: float
    vert_count: int
    edge_count: int


@dataclass(frozen=True)
class _ProjectedSpan2D:
    u_span: float
    v_span: float


@dataclass(frozen=True)
class _LoopClassificationResult:
    kind: LoopKind
    depth: int


@dataclass(frozen=True)
class _PatchNeighborChainRef:
    patch_id: int
    loop_index: int
    chain_index: int
    neighbor_patch_id: int
    start_vert_index: int
    end_vert_index: int

    @property
    def endpoint_pair(self) -> tuple[int, int]:
        return (
            min(self.start_vert_index, self.end_vert_index),
            max(self.start_vert_index, self.end_vert_index),
        )


@dataclass(frozen=True)
class _ResolvedLoopCornerIdentity:
    loop_vert_index: int
    vert_index: int
    vert_co: Vector
    resolved_exactly: bool = True


DirectionBucketKey = tuple[float, float, float]
PatchLoopKey = tuple[int, int]
CornerJunctionKey = tuple[int, int, int]


@dataclass(frozen=True)
class _FrameRunBuildEntry:
    dominant_role: FrameRole
    chain_indices: tuple[int, ...]


@dataclass(frozen=True)
class _LoopFrameRunBuildResult:
    effective_roles: tuple[FrameRole, ...]
    runs: tuple["_FrameRun", ...]


@dataclass(frozen=True)
class _FrameRunEndpointSpec:
    endpoint_kind: str
    corner_index: int


def _enum_value(value):
    return value.value if hasattr(value, "value") else value


def _coerce_face_indices(bm, faces_or_indices):
    bm.faces.ensure_lookup_table()

    ordered = []
    seen = set()
    for item in faces_or_indices or []:
        face_index = item.index if hasattr(item, "index") else int(item)
        if face_index in seen:
            continue
        if face_index < 0 or face_index >= len(bm.faces):
            continue
        seen.add(face_index)
        ordered.append(face_index)
    return ordered


def validate_solver_input_mesh(bm, face_indices, area_epsilon=1e-10):
    """Validate mesh topology before entering the solve pipeline."""

    bm.faces.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    checked_face_indices = tuple(_coerce_face_indices(bm, face_indices))
    report = MeshPreflightReport(checked_face_indices=checked_face_indices)
    if not checked_face_indices:
        return report

    relevant_face_set = set(checked_face_indices)
    face_signatures = {}
    for face in bm.faces:
        signature = tuple(sorted(vert.index for vert in face.verts))
        face_signatures.setdefault(signature, []).append(face.index)

    seen_duplicate_groups = set()
    for face_index in checked_face_indices:
        face = bm.faces[face_index]
        unique_vert_indices = tuple(sorted({vert.index for vert in face.verts}))
        if len(unique_vert_indices) < len(face.verts) or len(unique_vert_indices) < 3 or face.calc_area() <= area_epsilon:
            report.issues.append(
                MeshPreflightIssue(
                    code='DEGENERATE_FACE',
                    message=f'Face {face_index} is degenerate or repeats vertices',
                    face_indices=(face_index,),
                    vert_indices=unique_vert_indices,
                )
            )

        signature = tuple(sorted(vert.index for vert in face.verts))
        duplicate_faces = tuple(sorted(face_signatures.get(signature, [])))
        if len(duplicate_faces) > 1 and duplicate_faces not in seen_duplicate_groups:
            seen_duplicate_groups.add(duplicate_faces)
            report.issues.append(
                MeshPreflightIssue(
                    code='DUPLICATE_FACE',
                    message=f'Faces share identical vertex set: {list(duplicate_faces)}',
                    face_indices=duplicate_faces,
                    vert_indices=signature,
                )
            )

    visited_edges = set()
    for face_index in checked_face_indices:
        face = bm.faces[face_index]
        for edge in face.edges:
            if edge.index in visited_edges:
                continue
            visited_edges.add(edge.index)
            linked_faces = tuple(sorted(linked_face.index for linked_face in edge.link_faces))
            if len(linked_faces) <= 2:
                continue
            if not relevant_face_set.intersection(linked_faces):
                continue
            report.issues.append(
                MeshPreflightIssue(
                    code='NON_MANIFOLD_EDGE',
                    message=f'Edge {edge.index} has {len(linked_faces)} linked faces',
                    face_indices=linked_faces,
                    edge_indices=(edge.index,),
                    vert_indices=tuple(vert.index for vert in edge.verts),
                )
            )

    return report


def format_solver_input_preflight_report(report, mesh_name=None) -> FormattedReport:
    """Build text lines for blocking solve preflight issues."""

    lines = []
    if mesh_name:
        lines.append(f'Mesh: {mesh_name}')
    lines.append(f'Checked faces: {len(report.checked_face_indices)}')

    if report.is_valid:
        summary = f'Solver preflight: OK | faces:{len(report.checked_face_indices)}'
        return FormattedReport(lines=lines, summary=summary)

    issue_counts = {}
    for issue in report.issues:
        issue_counts[issue.code] = issue_counts.get(issue.code, 0) + 1

    lines.append('Blocking issues:')
    for issue in report.issues[:16]:
        refs = []
        if issue.face_indices:
            refs.append(f'faces:{list(issue.face_indices)}')
        if issue.edge_indices:
            refs.append(f'edges:{list(issue.edge_indices)}')
        if issue.vert_indices:
            refs.append(f'verts:{list(issue.vert_indices)}')
        suffix = f" | {' '.join(refs)}" if refs else ''
        lines.append(f'  - {issue.code}: {issue.message}{suffix}')

    remaining = len(report.issues) - 16
    if remaining > 0:
        lines.append(f'  - ... {remaining} more issues')

    parts = [f'{code}:{issue_counts[code]}' for code in sorted(issue_counts.keys())]
    summary = 'Solver preflight failed: ' + ', '.join(parts)
    return FormattedReport(lines=lines, summary=summary)

def get_expanded_islands(bm, initial_faces):
    """Build full/core islands for the two-pass unwrap pipeline."""

    initial_indices = _coerce_face_indices(bm, initial_faces)
    initial_set = set(initial_indices)
    visited = set()
    islands = []

    for start_idx in initial_indices:
        if start_idx in visited:
            continue

        full_faces = []
        core_faces = []
        stack = [start_idx]
        visited.add(start_idx)

        while stack:
            face_idx = stack.pop()
            face = bm.faces[face_idx]
            full_faces.append(face)
            if face_idx in initial_set:
                core_faces.append(face)

            for edge in face.edges:
                if edge.seam:
                    continue
                for neighbor in edge.link_faces:
                    neighbor_idx = neighbor.index
                    if neighbor_idx in visited:
                        continue
                    visited.add(neighbor_idx)
                    stack.append(neighbor_idx)

        islands.append({"full": full_faces, "core": core_faces})

    return islands


def _flood_fill_patches(bm, face_indices):
    """Flood-fill face groups split by seam and mesh borders."""

    face_indices = _coerce_face_indices(bm, face_indices)
    face_set = set(face_indices)
    visited = set()
    patches = []

    for start_idx in face_indices:
        if start_idx in visited:
            continue

        stack = [start_idx]
        visited.add(start_idx)
        patch = []

        while stack:
            face_idx = stack.pop()
            face = bm.faces[face_idx]
            patch.append(face_idx)

            for edge in face.edges:
                if edge.seam:
                    continue
                for neighbor in edge.link_faces:
                    neighbor_idx = neighbor.index
                    if neighbor_idx not in face_set or neighbor_idx in visited:
                        continue
                    visited.add(neighbor_idx)
                    stack.append(neighbor_idx)

        patches.append(patch)

    return patches


def _find_island_up(patch_faces, avg_normal):
    """Find a stable local up direction for patch classification."""

    edge_dirs: dict[DirectionBucketKey, dict[str, object]] = {}
    for face in patch_faces:
        for edge in face.edges:
            length = edge.calc_length()
            if length < 1e-4:
                continue
            vec = (edge.verts[1].co - edge.verts[0].co).normalized()
            if vec.dot(WORLD_UP) < 0.0:
                vec = -vec
            key: DirectionBucketKey = (round(vec.x, 2), round(vec.y, 2), round(vec.z, 2))
            if key not in edge_dirs:
                edge_dirs[key] = {"vec": vec, "weight": 0.0}
            edge_dirs[key]["weight"] += length

    best_direct_up = WORLD_UP.copy()
    max_up_score = -1.0
    for data in edge_dirs.values():
        vec = data["vec"]
        alignment = abs(vec.dot(WORLD_UP))
        score = data["weight"] * (alignment ** 2)
        if score > max_up_score:
            max_up_score = score
            best_direct_up = vec

    best_right = None
    max_right_score = -1.0
    for data in edge_dirs.values():
        vec = data["vec"]
        horizontal = 1.0 - abs(vec.dot(WORLD_UP))
        score = data["weight"] * (horizontal ** 2)
        if score > max_right_score:
            max_right_score = score
            best_right = vec

    if max_right_score > max_up_score and best_right is not None:
        derived_up = avg_normal.cross(best_right)
        if derived_up.dot(WORLD_UP) < 0.0:
            derived_up = -derived_up
        if derived_up.length_squared > 1e-6:
            return derived_up.normalized()

    if max_up_score > 0.0:
        return best_direct_up.normalized()
    return WORLD_UP.copy()


def _classify_patch(bm, face_indices):
    """Classify a patch as WALL, FLOOR, or SLOPE."""

    patch_faces = [bm.faces[idx] for idx in face_indices]
    patch_face_set = set(patch_faces)
    avg_normal = Vector((0.0, 0.0, 0.0))
    total_area = 0.0
    perimeter = 0.0

    for face in patch_faces:
        face_area = face.calc_area()
        avg_normal += face.normal * face_area
        total_area += face_area
        for edge in face.edges:
            link_count = sum(1 for linked_face in edge.link_faces if linked_face in patch_face_set)
            if link_count == 1:
                perimeter += edge.calc_length()

    if avg_normal.length > 0.0:
        avg_normal.normalize()
    else:
        avg_normal = Vector((0.0, 0.0, 1.0))

    signed_up_dot = avg_normal.dot(WORLD_UP)
    up_dot = abs(signed_up_dot)
    if up_dot > FLOOR_THRESHOLD:
        patch_type = PatchType.FLOOR
    elif up_dot < WALL_THRESHOLD:
        patch_type = PatchType.WALL
    else:
        patch_type = PatchType.SLOPE

    if signed_up_dot > WALL_THRESHOLD:
        world_facing = WorldFacing.UP
    elif signed_up_dot < -WALL_THRESHOLD:
        world_facing = WorldFacing.DOWN
    else:
        world_facing = WorldFacing.SIDE

    return patch_type, world_facing, avg_normal, total_area, perimeter


def _calc_surface_basis(normal, ref_up=WORLD_UP):
    up_proj = ref_up - normal * ref_up.dot(normal)
    if up_proj.length_squared < 1e-5:
        tangent = Vector((1.0, 0.0, 0.0))
        tangent = (tangent - normal * tangent.dot(normal)).normalized()
        return tangent, normal.cross(tangent).normalized()
    bitangent = up_proj.normalized()
    return bitangent.cross(normal).normalized(), bitangent


def _build_patch_basis(bm, face_indices, patch_type, normal):
    """Build a local orthogonal basis for the patch."""

    patch_faces = [bm.faces[idx] for idx in face_indices]
    island_up = _find_island_up(patch_faces, normal)

    if patch_type in {PatchType.WALL, PatchType.SLOPE}:
        up_proj = WORLD_UP - normal * WORLD_UP.dot(normal)
        if up_proj.length_squared > 1e-8:
            basis_v = up_proj.normalized()
            basis_u = basis_v.cross(normal).normalized()
        else:
            basis_u, basis_v = _calc_surface_basis(normal, island_up)
    else:
        sorted_faces = sorted(
            patch_faces,
            key=lambda face: face.calc_area() * (max(0.0, face.normal.dot(normal)) ** 4),
            reverse=True,
        )
        seed_face = sorted_faces[0] if sorted_faces else patch_faces[0]
        basis_u, basis_v = _calc_surface_basis(seed_face.normal, island_up)

    return basis_u, basis_v


def _is_boundary_side(loop, patch_face_indices):
    """Check whether the loop side belongs to the patch boundary."""

    edge = loop.edge
    in_patch_count = sum(1 for linked_face in edge.link_faces if linked_face.index in patch_face_indices)
    if len(edge.link_faces) == 1:
        return True
    if in_patch_count == 1:
        return True
    if in_patch_count >= 2 and edge.seam:
        return True
    return False


def _boundary_side_key(loop):
    return _BoundarySideKey(
        face_index=loop.face.index,
        edge_index=loop.edge.index,
        vert_index=loop.vert.index,
    )


def _find_next_boundary_side(loop, patch_face_indices):
    """Walk to the next boundary side around the same patch."""

    candidate = loop.link_loop_next
    safety = 0
    while safety < 200000:
        safety += 1
        if _is_boundary_side(candidate, patch_face_indices):
            return candidate

        radial = candidate.link_loop_radial_next
        if radial == candidate:
            return None

        next_radial = None
        probe = radial
        while True:
            if probe.face.index in patch_face_indices:
                next_radial = probe
                break
            probe = probe.link_loop_radial_next
            if probe == candidate:
                break

        if next_radial is None:
            return None

        candidate = next_radial.link_loop_next

    return None


def _trace_boundary_loops(patch_faces):
    """Trace ordered boundary loops for one patch."""

    patch_face_indices = {face.index for face in patch_faces}
    raw_loops = []
    used_sides = set()

    for face in patch_faces:
        for loop in face.loops:
            if not _is_boundary_side(loop, patch_face_indices):
                continue

            start_key = _boundary_side_key(loop)
            if start_key in used_sides:
                continue

            current = loop
            vert_indices = []
            vert_cos = []
            edge_indices = []
            side_face_indices = []
            closed = False
            safety = 0

            while safety < 200000:
                safety += 1
                current_key = _boundary_side_key(current)
                if current_key in used_sides:
                    break

                used_sides.add(current_key)
                vert_indices.append(current.vert.index)
                vert_cos.append(current.vert.co.copy())
                edge_indices.append(current.edge.index)
                side_face_indices.append(current.face.index)

                next_side = _find_next_boundary_side(current, patch_face_indices)
                if next_side is None:
                    break
                if _boundary_side_key(next_side) == start_key:
                    closed = True
                    break

                current = next_side

            if edge_indices:
                raw_loops.append(
                    _RawBoundaryLoop(
                        vert_indices=vert_indices,
                        vert_cos=vert_cos,
                        edge_indices=edge_indices,
                        side_face_indices=side_face_indices,
                        kind=LoopKind.OUTER,
                        depth=0,
                        closed=closed,
                    )
                )

    return raw_loops


def _neighbor_for_side(edge_index, side_face_index, patch_face_indices, face_to_patch, patch_id, bm):
    """Resolve the neighbor id for one boundary side."""

    edge = bm.edges[edge_index]
    if len(edge.link_faces) == 1:
        return NB_MESH_BORDER

    in_patch_faces = [linked_face for linked_face in edge.link_faces if linked_face.index in patch_face_indices]
    if len(in_patch_faces) >= 2 and edge.seam:
        return NB_SEAM_SELF

    other_faces = [linked_face for linked_face in edge.link_faces if linked_face.index not in patch_face_indices]
    if not other_faces:
        return NB_MESH_BORDER

    neighbor_patch_id = face_to_patch.get(other_faces[0].index, NB_MESH_BORDER)
    if neighbor_patch_id == patch_id:
        return NB_SEAM_SELF

    return neighbor_patch_id


def _split_loop_into_chains_by_neighbor(raw_loop, loop_neighbors):
    """Split a boundary loop into chains when the neighbor changes."""

    loop_vert_indices = raw_loop.vert_indices
    loop_vert_cos = raw_loop.vert_cos
    loop_edge_indices = raw_loop.edge_indices
    loop_side_face_indices = raw_loop.side_face_indices
    vertex_count = len(loop_vert_cos)
    edge_count = len(loop_edge_indices)
    if edge_count == 0:
        return []

    neighbors = list(loop_neighbors[:edge_count])
    if len(neighbors) < edge_count:
        neighbors.extend([NB_MESH_BORDER] * (edge_count - len(neighbors)))

    split_indices = []
    for idx in range(vertex_count):
        if neighbors[(idx - 1) % edge_count] != neighbors[idx % edge_count]:
            split_indices.append(idx)

    if not split_indices:
        return [
            _RawBoundaryChain(
                vert_indices=list(loop_vert_indices),
                vert_cos=list(loop_vert_cos),
                edge_indices=list(loop_edge_indices),
                side_face_indices=list(loop_side_face_indices),
                neighbor=neighbors[0],
                is_closed=True,
                start_loop_index=0,
                end_loop_index=0,
            )
        ]

    chains = []
    split_count = len(split_indices)
    for split_idx in range(split_count):
        v_start = split_indices[split_idx]
        v_end = split_indices[(split_idx + 1) % split_count]

        chain_vert_indices = []
        chain_vert_cos = []
        chain_edge_indices = []
        chain_side_face_indices = []
        idx = v_start
        safety = 0
        while safety < vertex_count + 2:
            safety += 1
            chain_vert_indices.append(loop_vert_indices[idx % vertex_count])
            chain_vert_cos.append(loop_vert_cos[idx % vertex_count])
            chain_edge_indices.append(loop_edge_indices[idx % edge_count])
            chain_side_face_indices.append(loop_side_face_indices[idx % vertex_count])
            idx += 1
            if idx % vertex_count == v_end % vertex_count:
                chain_vert_indices.append(loop_vert_indices[v_end % vertex_count])
                chain_vert_cos.append(loop_vert_cos[v_end % vertex_count])
                chain_side_face_indices.append(loop_side_face_indices[v_end % vertex_count])
                break

        chains.append(
            _RawBoundaryChain(
                vert_indices=chain_vert_indices,
                vert_cos=chain_vert_cos,
                edge_indices=chain_edge_indices,
                side_face_indices=chain_side_face_indices,
                neighbor=neighbors[v_start % edge_count],
                is_closed=False,
                start_loop_index=v_start % vertex_count,
                end_loop_index=v_end % vertex_count,
            )
        )

    return chains


def _build_boundary_loop_base(raw_loop):
    """Materialize the final BoundaryLoop shell before chain/corner population."""

    loop_kind = raw_loop.kind
    if not isinstance(loop_kind, LoopKind):
        loop_kind = LoopKind(loop_kind)

    return BoundaryLoop(
        vert_indices=list(raw_loop.vert_indices),
        vert_cos=[co.copy() for co in raw_loop.vert_cos],
        edge_indices=list(raw_loop.edge_indices),
        side_face_indices=list(raw_loop.side_face_indices),
        kind=loop_kind,
        depth=int(raw_loop.depth),
    )


def _collect_raw_loop_neighbors(raw_loop, patch_face_indices, face_to_patch, patch_id, bm):
    """Collect per-side neighbor ids for one raw loop."""

    return [
        _neighbor_for_side(edge_index, side_face_index, patch_face_indices, face_to_patch, patch_id, bm)
        for edge_index, side_face_index in zip(raw_loop.edge_indices, raw_loop.side_face_indices)
    ]


def _begin_boundary_loop_build(raw_loop, patch_face_indices, face_to_patch, patch_id, bm):
    """Create the typed initial state for one loop before refinement."""

    state = _BoundaryLoopBuildState(
        raw_loop=raw_loop,
        boundary_loop=_build_boundary_loop_base(raw_loop),
    )
    state.loop_neighbors = _collect_raw_loop_neighbors(
        raw_loop,
        patch_face_indices,
        face_to_patch,
        patch_id,
        bm,
    )
    state.raw_chains = _split_loop_into_chains_by_neighbor(raw_loop, state.loop_neighbors)
    return state


def _refine_boundary_loop_raw_chains(state, basis_u, basis_v, bm):
    """Run split/merge refinement over the raw-chain stage of one loop."""

    state.raw_chains = _merge_bevel_wrap_chains(state.raw_chains, bm)
    state.raw_chains = _try_geometric_outer_loop_split(
        state.raw_loop,
        state.raw_chains,
        basis_u,
        basis_v,
        bm=bm,
    )
    state.raw_chains = _split_border_chains_by_corners(
        state.raw_chains,
        basis_u,
        basis_v,
        bm=bm,
    )


def _derive_boundary_loop_topology(state, basis_u, basis_v, patch_id, loop_index):
    """Derive final chains and corners only from the post-refinement loop state."""

    boundary_loop = state.boundary_loop
    chains = _build_boundary_chain_objects(state.raw_chains, basis_u, basis_v)
    _downgrade_same_role_point_contact_chains(chains, basis_u, basis_v, patch_id, loop_index)
    chains = _merge_same_role_border_chains(chains)
    boundary_loop.chains = chains

    corners = _derive_loop_corners_from_final_chains(
        boundary_loop,
        basis_u,
        basis_v,
        patch_id,
        loop_index,
    )
    return _BoundaryLoopDerivedTopology(
        chains=chains,
        corners=corners,
        uses_geometric_corner_fallback=(len(chains) < 2),
    )


def _finalize_boundary_loop_build(state, basis_u, basis_v, patch_id, loop_index):
    """Finalize chains, corners and endpoint topology from a loop build state."""

    boundary_loop = state.boundary_loop
    derived_topology = _derive_boundary_loop_topology(state, basis_u, basis_v, patch_id, loop_index)
    boundary_loop.chains = derived_topology.chains
    boundary_loop.corners = derived_topology.corners
    _assign_loop_chain_endpoint_topology(boundary_loop)
    _validate_boundary_loop_topology(boundary_loop, patch_id, loop_index)
    return boundary_loop


def _merge_bevel_wrap_chains(chains, bm):
    """Preserve the primary neighbor split contract.

    Исторически здесь был merge через смену соседа, если поворот считался
    bevel-wrap. Но это ломает базовый topology invariant:
    `BoundaryChain` должен иметь одного соседа, а primary split по neighbor
    является authoritative. Если после split два raw chains имеют разные
    `neighbor`, склеивать их назад уже нельзя без разрушения topology layer.

    Поэтому bevel-wrap merge больше не имеет права менять результат
    primary split. Любой будущий bevel-specific cleanup должен работать
    без нарушения neighbor contract.
    """
    return list(chains)


def _signed_area_2d(poly):
    area = 0.0
    count = len(poly)
    for idx in range(count):
        x1, y1 = poly[idx].x, poly[idx].y
        x2, y2 = poly[(idx + 1) % count].x, poly[(idx + 1) % count].y
        area += x1 * y2 - x2 * y1
    return 0.5 * area


def _point_in_polygon_2d(point, poly):
    x, y = point.x, point.y
    inside = False
    count = len(poly)
    for idx in range(count):
        x1, y1 = poly[idx].x, poly[idx].y
        x2, y2 = poly[(idx + 1) % count].x, poly[(idx + 1) % count].y
        if (y1 > y) != (y2 > y):
            x_intersection = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-30) + x1
            if x < x_intersection:
                inside = not inside
    return inside


def _select_polygon_interior_point(poly):
    if len(poly) < 3:
        return poly[0] if poly else _PlanarPoint2D(0.0, 0.0)

    signed_area = _signed_area_2d(poly)
    edges_by_len: list[_PolygonEdgeLengthCandidate] = []
    for idx in range(len(poly)):
        next_idx = (idx + 1) % len(poly)
        dx = poly[next_idx].x - poly[idx].x
        dy = poly[next_idx].y - poly[idx].y
        edges_by_len.append(_PolygonEdgeLengthCandidate(dx * dx + dy * dy, idx))
    edges_by_len.sort(reverse=True)

    for edge_candidate in edges_by_len:
        next_idx = (edge_candidate.index + 1) % len(poly)
        mid_x = (poly[edge_candidate.index].x + poly[next_idx].x) * 0.5
        mid_y = (poly[edge_candidate.index].y + poly[next_idx].y) * 0.5
        dx = poly[next_idx].x - poly[edge_candidate.index].x
        dy = poly[next_idx].y - poly[edge_candidate.index].y
        edge_len = math.sqrt(edge_candidate.len_squared)
        if edge_len < 1e-12:
            continue

        if signed_area >= 0.0:
            normal_x, normal_y = -dy / edge_len, dx / edge_len
        else:
            normal_x, normal_y = dy / edge_len, -dx / edge_len

        epsilon = edge_len * 0.01
        point = _PlanarPoint2D(mid_x + normal_x * epsilon, mid_y + normal_y * epsilon)
        if _point_in_polygon_2d(point, poly):
            return point

    return _PlanarPoint2D(
        sum(point.x for point in poly) / len(poly),
        sum(point.y for point in poly) / len(poly),
    )


def _project_raw_loop_to_patch_plane(raw_loop, basis_u, basis_v):
    return [
        _PlanarPoint2D(point.dot(basis_u), point.dot(basis_v))
        for point in raw_loop.vert_cos
    ]


def _classify_raw_loops_by_nesting_depth(raw_loops, basis_u, basis_v):
    """Classify raw loops as OUTER or HOLE via planar nesting in patch-local basis."""

    if not raw_loops:
        return

    if len(raw_loops) == 1:
        raw_loops[0].kind = LoopKind.OUTER
        raw_loops[0].depth = 0
        return

    polys_2d = [
        _project_raw_loop_to_patch_plane(raw_loop, basis_u, basis_v)
        for raw_loop in raw_loops
    ]
    interior_points = [_select_polygon_interior_point(poly) for poly in polys_2d]

    for loop_index, raw_loop in enumerate(raw_loops):
        depth = 0
        for poly_index, poly in enumerate(polys_2d):
            if loop_index == poly_index:
                continue
            if _point_in_polygon_2d(interior_points[loop_index], poly):
                depth += 1

        raw_loop.depth = depth
        raw_loop.kind = LoopKind.OUTER if depth % 2 == 0 else LoopKind.HOLE


def _prepare_outer_hole_classification_inputs(raw_patch_data):
    """Normalize trivial loop kinds and collect only multi-loop patches for classification."""

    classification_inputs = []

    for patch_data in raw_patch_data.values():
        if len(patch_data.raw_loops) <= 1:
            for raw_loop in patch_data.raw_loops:
                raw_loop.kind = LoopKind.OUTER
                raw_loop.depth = 0
            continue
        classification_inputs.append(patch_data)

    return classification_inputs


def _classify_raw_loops_via_uv(raw_loops, bm, patch_face_indices, uv_layer):
    """Classify raw loops as OUTER or HOLE using temporary UV data."""

    if not raw_loops:
        return

    if len(raw_loops) == 1:
        raw_loops[0].kind = LoopKind.OUTER
        raw_loops[0].depth = 0
        return

    patch_face_indices = set(patch_face_indices)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    def get_side_uv(face_index, edge_index, vert_index):
        face = bm.faces[face_index]
        for loop in face.loops:
            if loop.edge.index == edge_index and loop.vert.index == vert_index:
                uv = loop[uv_layer].uv
                return _PlanarPoint2D(uv.x, uv.y)

        vert = bm.verts[vert_index]
        uvs = [loop[uv_layer].uv for loop in vert.link_loops if loop.face.index in patch_face_indices]
        if not uvs:
            return _PlanarPoint2D(0.0, 0.0)
        return _PlanarPoint2D(
            sum(uv.x for uv in uvs) / len(uvs),
            sum(uv.y for uv in uvs) / len(uvs),
        )

    polys_2d = []
    for raw_loop in raw_loops:
        poly = []
        for face_index, edge_index, vert_index in zip(
            raw_loop.side_face_indices,
            raw_loop.edge_indices,
            raw_loop.vert_indices,
        ):
            poly.append(get_side_uv(face_index, edge_index, vert_index))
        polys_2d.append(poly)

    interior_points = [_select_polygon_interior_point(poly) for poly in polys_2d]

    for loop_index, raw_loop in enumerate(raw_loops):
        depth = 0
        for poly_index, poly in enumerate(polys_2d):
            if loop_index == poly_index:
                continue
            if _point_in_polygon_2d(interior_points[loop_index], poly):
                depth += 1

        raw_loop.depth = depth
        raw_loop.kind = LoopKind.OUTER if depth == 0 or depth % 2 == 0 else LoopKind.HOLE


def _set_face_selection(bm, face_indices):
    """Apply face selection explicitly for the temporary analysis UV boundary."""

    bm.faces.ensure_lookup_table()
    face_index_set = set(face_indices)
    for face in bm.faces:
        face.select = face.index in face_index_set


def _allocate_analysis_temp_uv_layer(bm, obj, base_name="_cftuv_temp_outer_hole"):
    """Create a fresh temporary UV layer so rollback never touches persistent user layers."""

    uv_name = base_name
    suffix = 1
    while bm.loops.layers.uv.get(uv_name) is not None or uv_name in obj.data.uv_layers:
        uv_name = f"{base_name}_{suffix}"
        suffix += 1

    return uv_name, bm.loops.layers.uv.new(uv_name)


def _begin_analysis_uv_classification(bm, obj):
    """Enter the explicit UV-dependent OUTER/HOLE analysis boundary."""

    state = _AnalysisUvClassificationState(
        original_active_uv_name=obj.data.uv_layers.active.name if obj.data.uv_layers.active else None,
        original_selection=[face.index for face in bm.faces if face.select],
    )
    state.temp_uv_name, uv_layer = _allocate_analysis_temp_uv_layer(bm, obj)

    try:
        bmesh.update_edit_mesh(obj.data)
        if state.temp_uv_name in obj.data.uv_layers:
            obj.data.uv_layers[state.temp_uv_name].active = True

        _set_face_selection(bm, [])
    except Exception:
        _finish_analysis_uv_classification(bm, obj, uv_layer, state)
        raise

    return uv_layer, state


def _finish_analysis_uv_classification(bm, obj, uv_layer, state):
    """Restore mesh selection and active UV after temporary OUTER/HOLE classification."""

    if uv_layer is not None:
        bm.loops.layers.uv.remove(uv_layer)

    _set_face_selection(bm, state.original_selection)
    bmesh.update_edit_mesh(obj.data)

    if state.original_active_uv_name and state.original_active_uv_name in obj.data.uv_layers:
        obj.data.uv_layers[state.original_active_uv_name].active = True


def _unwrap_patch_faces_for_loop_classification(bm, obj, patch_face_indices):
    """Run temporary unwrap for one patch inside the analysis UV boundary."""

    _set_face_selection(bm, patch_face_indices)
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.uv.unwrap(method="CONFORMAL", fill_holes=False, margin=0.0)
    _set_face_selection(bm, [])


def _classify_multi_loop_patches_via_uv(bm, classification_inputs, obj):
    """Execute the isolated UV-dependent loop-kind pass for multi-loop patches only."""

    uv_layer = None
    state = None

    try:
        uv_layer, state = _begin_analysis_uv_classification(bm, obj)
        for patch_data in classification_inputs:
            _unwrap_patch_faces_for_loop_classification(bm, obj, patch_data.face_indices)
            _classify_raw_loops_via_uv(patch_data.raw_loops, bm, patch_data.face_indices, uv_layer)
    finally:
        if uv_layer is not None and state is not None:
            _finish_analysis_uv_classification(bm, obj, uv_layer, state)


def _classify_multi_loop_patches_by_nesting(classification_inputs):
    """Diagnostics-only planar OUTER/HOLE classification helper."""

    for patch_data in classification_inputs:
        _classify_raw_loops_by_nesting_depth(
            patch_data.raw_loops,
            patch_data.basis_u,
            patch_data.basis_v,
        )


def _snapshot_planar_loop_classification(raw_loops, basis_u, basis_v):
    """Run planar loop classification on shadow copies for diagnostics only."""

    shadow_loops = [
        _RawBoundaryLoop(
            vert_indices=list(raw_loop.vert_indices),
            vert_cos=list(raw_loop.vert_cos),
            edge_indices=list(raw_loop.edge_indices),
            side_face_indices=list(raw_loop.side_face_indices),
            kind=raw_loop.kind,
            depth=raw_loop.depth,
            closed=raw_loop.closed,
        )
        for raw_loop in raw_loops
    ]
    _classify_raw_loops_by_nesting_depth(shadow_loops, basis_u, basis_v)
    return tuple(
        _LoopClassificationResult(kind=shadow_loop.kind, depth=shadow_loop.depth)
        for shadow_loop in shadow_loops
    )


def _measure_chain_axis_metrics(chain_vert_cos, basis_u, basis_v):
    """Measure asymmetric H/V alignment in full local 3D basis.

    H_FRAME: plane-based, chain should stay close to the local N-U plane.
    V_FRAME: axis-based, chain should stay close to the local V axis.
    """

    if len(chain_vert_cos) < 2:
        return None

    basis_n = basis_u.cross(basis_v)
    if basis_n.length_squared < 1e-8:
        return None
    basis_n.normalize()

    total_length = 0.0
    h_deviation_sum = 0.0
    h_deviation_max = 0.0
    h_support = 0.0
    v_deviation_sum = 0.0
    v_deviation_max = 0.0
    v_support = 0.0

    for point_index in range(len(chain_vert_cos) - 1):
        delta = chain_vert_cos[point_index + 1] - chain_vert_cos[point_index]
        seg_len = delta.length
        if seg_len < 1e-8:
            continue

        dir_u = delta.dot(basis_u) / seg_len
        dir_v = delta.dot(basis_v) / seg_len
        dir_n = delta.dot(basis_n) / seg_len

        h_deviation = abs(dir_v)
        h_support_component = math.sqrt(max(0.0, dir_u * dir_u + dir_n * dir_n))

        v_deviation = h_support_component
        v_support_component = abs(dir_v)

        total_length += seg_len
        h_deviation_sum += seg_len * h_deviation
        h_deviation_max = max(h_deviation_max, h_deviation)
        h_support += seg_len * h_support_component
        v_deviation_sum += seg_len * v_deviation
        v_deviation_max = max(v_deviation_max, v_deviation)
        v_support += seg_len * v_support_component

    if total_length < 1e-6:
        return None

    return {
        "total_length": total_length,
        "h_avg_deviation": h_deviation_sum / total_length,
        "h_max_deviation": h_deviation_max,
        "h_support": h_support,
        "v_avg_deviation": v_deviation_sum / total_length,
        "v_max_deviation": v_deviation_max,
        "v_support": v_support,
    }


def _classify_chain_frame_role(
    chain_vert_cos,
    basis_u,
    basis_v,
    threshold_h=FRAME_ALIGNMENT_THRESHOLD_H,
    threshold_v=FRAME_ALIGNMENT_THRESHOLD_V,
    strict_guards=True,
):
    """Classify chain with asymmetric H/V semantics.

    H_FRAME: must stay close to the local N-U plane.
    V_FRAME: must stay close to the local V axis.
    """

    metrics = _measure_chain_axis_metrics(chain_vert_cos, basis_u, basis_v)
    if metrics is None:
        return FrameRole.FREE

    if strict_guards:
        h_max_deviation_limit = max(threshold_h * 2.0, threshold_h + 0.03)
        v_max_deviation_limit = max(threshold_v * 2.0, threshold_v + 0.03)
    else:
        h_max_deviation_limit = max(threshold_h * 4.0, threshold_h + 0.08)
        v_max_deviation_limit = max(threshold_v * 4.0, threshold_v + 0.08)

    h_ok = (
        metrics["h_support"] > 1e-6
        and metrics["h_avg_deviation"] < threshold_h
        and metrics["h_max_deviation"] < h_max_deviation_limit
    )
    v_ok = (
        metrics["v_support"] > 1e-6
        and metrics["v_avg_deviation"] < threshold_v
        and metrics["v_max_deviation"] < v_max_deviation_limit
    )

    if h_ok and v_ok:
        if abs(metrics["h_avg_deviation"] - metrics["v_avg_deviation"]) > 1e-6:
            return FrameRole.H_FRAME if metrics["h_avg_deviation"] < metrics["v_avg_deviation"] else FrameRole.V_FRAME
        return FrameRole.H_FRAME if metrics["h_support"] >= metrics["v_support"] else FrameRole.V_FRAME
    if h_ok:
        return FrameRole.H_FRAME
    if v_ok:
        return FrameRole.V_FRAME
    return FrameRole.FREE


def _find_corner_reference_point(points, corner_co, reverse=False):
    """Find a non-degenerate neighbor point around a chain junction."""

    if reverse:
        candidates = list(reversed(points[:-1]))
    else:
        candidates = list(points[1:])

    for point in candidates:
        if (point - corner_co).length_squared > 1e-12:
            return point
    return None



def _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v):
    """Measure the turn angle at a chain junction in patch-local 2D space."""

    if prev_point is None or next_point is None:
        return 0.0

    prev_vec_3d = corner_co - prev_point
    next_vec_3d = next_point - corner_co
    prev_vec = Vector((prev_vec_3d.dot(basis_u), prev_vec_3d.dot(basis_v)))
    next_vec = Vector((next_vec_3d.dot(basis_u), next_vec_3d.dot(basis_v)))
    if prev_vec.length_squared < 1e-12 or next_vec.length_squared < 1e-12:
        return 0.0

    return math.degrees(prev_vec.angle(next_vec, 0.0))


def _measure_corner_turn_angle_3d(corner_co, prev_point, next_point):
    """Measure the geometric turn angle in full 3D space."""

    if prev_point is None or next_point is None:
        return 0.0

    prev_vec = corner_co - prev_point
    next_vec = next_point - corner_co
    if prev_vec.length_squared < 1e-12 or next_vec.length_squared < 1e-12:
        return 0.0

    return math.degrees(prev_vec.angle(next_vec, 0.0))



def _build_geometric_loop_corners(boundary_loop, basis_u, basis_v):
    """Build geometric corners directly from loop turns, even inside one FREE chain."""

    vertex_count = len(boundary_loop.vert_cos)
    if vertex_count < 3:
        return []

    corners = []
    for loop_vert_index in range(vertex_count):
        corner_co = boundary_loop.vert_cos[loop_vert_index].copy()
        prev_point = boundary_loop.vert_cos[(loop_vert_index - 1) % vertex_count]
        next_point = boundary_loop.vert_cos[(loop_vert_index + 1) % vertex_count]
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)
        if turn_angle_deg < CORNER_ANGLE_THRESHOLD_DEG:
            continue
        corners.append(
            BoundaryCorner(
                loop_vert_index=loop_vert_index,
                vert_index=boundary_loop.vert_indices[loop_vert_index] if loop_vert_index < len(boundary_loop.vert_indices) else -1,
                vert_co=corner_co,
                prev_chain_index=0,
                next_chain_index=0,
                corner_kind=CornerKind.GEOMETRIC,
                turn_angle_deg=turn_angle_deg,
                prev_role=FrameRole.FREE,
                next_role=FrameRole.FREE,
            )
        )

    return corners



def _loop_arc_length(loop_vert_cos, start_loop_index, end_loop_index):
    """Measure the closed-loop arc length from start to end."""

    vertex_count = len(loop_vert_cos)
    if vertex_count < 2:
        return 0.0

    length = 0.0
    loop_index = start_loop_index % vertex_count
    safety = 0
    while safety < vertex_count + 1:
        safety += 1
        next_loop_index = (loop_index + 1) % vertex_count
        length += (loop_vert_cos[next_loop_index] - loop_vert_cos[loop_index]).length
        loop_index = next_loop_index
        if loop_index == end_loop_index % vertex_count:
            break

    return length


def _is_tangent_plane_turn(corner_co, prev_point, next_point, vert_normal):
    """Проверяет, что поворот в вершине происходит в касательной плоскости (UV-угол),
    а не вдоль нормали (бевель-заворот поверхности).

    UV-угол: ось поворота ≈ параллельна нормали вершины → cross ∥ normal
    Бевель-заворот: ось поворота ≈ перпендикулярна нормали → cross ⊥ normal

    Используем vertex normal (BMVert.normal) — усреднённая по всем смежным faces,
    стабильна в точке стыка разно-ориентированных граней.
    Edges нормализуются перед cross product — иначе для коротких бевель-edges
    cross product слишком мал и проваливается в collinear branch.
    """
    edge_in = corner_co - prev_point
    edge_out = next_point - corner_co
    if edge_in.length_squared < 1e-12 or edge_out.length_squared < 1e-12:
        return True  # дегенеративный случай — пропускаем фильтр
    edge_in_n = edge_in.normalized()
    edge_out_n = edge_out.normalized()
    turn_axis = edge_in_n.cross(edge_out_n)
    if turn_axis.length_squared < 1e-12:
        # Коллинеарные edges (turn≈0° или ≈180°). Для hairpin (≈180°)
        # это скорее бевель-заворот — отфильтруем.
        dot_val = edge_in_n.dot(edge_out_n)
        if dot_val < -0.5:
            return False
        return True
    # Доля оси поворота, совпадающая с нормалью вершины
    alignment = abs(turn_axis.normalized().dot(vert_normal.normalized()))
    # alignment ≈ 1.0 → поворот в касательной плоскости (UV-угол)
    # alignment ≈ 0.0 → поворот вдоль нормали (бевель)
    return alignment > 0.3


def _collect_geometric_split_indices(loop_vert_cos, basis_u, basis_v,
                                     loop_vert_indices=None, bm=None):
    """Compatibility wrapper over the unified closed-loop corner detector."""

    return _detect_closed_loop_corner_indices(
        loop_vert_cos,
        basis_u,
        basis_v,
        loop_vert_indices=loop_vert_indices,
        bm=bm,
    )


def _split_closed_loop_by_corner_indices(raw_loop, split_indices, neighbor_patch_id):
    """Split one closed raw loop into raw chains at geometric corners."""

    loop_vert_indices = list(raw_loop.vert_indices)
    loop_vert_cos = list(raw_loop.vert_cos)
    loop_edge_indices = list(raw_loop.edge_indices)
    loop_side_face_indices = list(raw_loop.side_face_indices)
    vertex_count = len(loop_vert_cos)
    edge_count = len(loop_edge_indices)
    if vertex_count < 2 or edge_count == 0:
        return []

    ordered_split_indices = sorted({loop_vert_index % vertex_count for loop_vert_index in split_indices})
    if len(ordered_split_indices) < 2:
        return []

    raw_chains = []
    split_count = len(ordered_split_indices)
    for split_index in range(split_count):
        start_loop_index = ordered_split_indices[split_index]
        end_loop_index = ordered_split_indices[(split_index + 1) % split_count]

        chain_vert_indices = []
        chain_vert_cos = []
        chain_edge_indices = []
        chain_side_face_indices = []

        loop_index = start_loop_index
        safety = 0
        while safety < vertex_count + 2:
            safety += 1
            chain_vert_indices.append(loop_vert_indices[loop_index % vertex_count])
            chain_vert_cos.append(loop_vert_cos[loop_index % vertex_count])
            chain_edge_indices.append(loop_edge_indices[loop_index % edge_count])
            chain_side_face_indices.append(loop_side_face_indices[loop_index % vertex_count])
            loop_index += 1
            if loop_index % vertex_count == end_loop_index % vertex_count:
                chain_vert_indices.append(loop_vert_indices[end_loop_index % vertex_count])
                chain_vert_cos.append(loop_vert_cos[end_loop_index % vertex_count])
                chain_side_face_indices.append(loop_side_face_indices[end_loop_index % vertex_count])
                break

        if len(chain_vert_indices) < 2:
            continue

        raw_chains.append(
            _RawBoundaryChain(
                vert_indices=chain_vert_indices,
                vert_cos=chain_vert_cos,
                edge_indices=chain_edge_indices,
                side_face_indices=chain_side_face_indices,
                neighbor=neighbor_patch_id,
                is_closed=False,
                start_loop_index=start_loop_index,
                end_loop_index=end_loop_index,
            )
        )

    return raw_chains


def _find_open_chain_corners(vert_cos, basis_u, basis_v, min_spacing=2,
                             vert_indices=None, bm=None):
    """Compatibility wrapper over the unified open-border corner detector."""

    return _detect_open_border_corner_indices(
        vert_cos,
        basis_u,
        basis_v,
        min_spacing=min_spacing,
        vert_indices=vert_indices,
        bm=bm,
    )


def _find_open_chain_corners_filtered(vert_cos, basis_u, basis_v, min_spacing=2,
                                      vert_indices=None, bm=None):
    """Compatibility wrapper over the unified open-border corner detector."""

    return _detect_open_border_corner_indices(
        vert_cos,
        basis_u,
        basis_v,
        min_spacing=min_spacing,
        vert_indices=vert_indices,
        bm=bm,
    )


def _collect_corner_turn_candidates_shared(polyline_cos, basis_u, basis_v, policy,
                                           vert_indices=None, bm=None):
    """Shared corner candidate collection for both closed loops and open border chains."""

    vertex_count = len(polyline_cos)
    if vertex_count < (4 if policy.closed_loop else 3):
        return [], []

    use_vert_normals = (vert_indices is not None and bm is not None)
    if use_vert_normals:
        bm.verts.ensure_lookup_table()

    raw_candidates: list[_CornerTurnCandidate] = []
    filtered_candidates: list[_CornerTurnCandidate] = []
    candidate_indices = range(vertex_count) if policy.closed_loop else range(1, vertex_count - 1)

    for point_index in candidate_indices:
        prev_point = polyline_cos[(point_index - 1) % vertex_count]
        corner_co = polyline_cos[point_index]
        next_point = polyline_cos[(point_index + 1) % vertex_count]
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)
        print(
            f"[CFTUV][CornerDetect] idx={point_index} turn={turn_angle_deg:.1f} "
            f"threshold={CORNER_ANGLE_THRESHOLD_DEG} use_normals={use_vert_normals}"
        )
        if turn_angle_deg < CORNER_ANGLE_THRESHOLD_DEG:
            continue

        candidate = _CornerTurnCandidate(point_index, turn_angle_deg)
        raw_candidates.append(candidate)
        is_hairpin_turn = turn_angle_deg >= 150.0

        if policy.reject_projected_hairpins and is_hairpin_turn:
            turn_angle_3d = _measure_corner_turn_angle_3d(corner_co, prev_point, next_point)
            if turn_angle_3d < 120.0:
                print(
                    f"[CFTUV][CornerDetect] skip idx={point_index} reason=projected_hairpin "
                    f"turn2d={turn_angle_deg:.1f} turn3d={turn_angle_3d:.1f}"
                )
                continue

        should_filter_bevel_turn = (
            policy.filter_all_bevel_turns
            or (policy.filter_hairpin_bevel_turns and is_hairpin_turn)
        )
        if use_vert_normals and should_filter_bevel_turn:
            vert_index = vert_indices[point_index]
            if vert_index < len(bm.verts):
                vert_normal = bm.verts[vert_index].normal
                if vert_normal.length_squared > 1e-12 and not _is_tangent_plane_turn(
                    corner_co,
                    prev_point,
                    next_point,
                    vert_normal,
                ):
                    reason = "hairpin_not_tangent" if is_hairpin_turn else "bevel_not_tangent"
                    print(
                        f"[CFTUV][CornerDetect] skip idx={point_index} reason={reason} "
                        f"turn2d={turn_angle_deg:.1f}"
                    )
                    continue

        filtered_candidates.append(candidate)

    return raw_candidates, filtered_candidates


def _filter_corner_candidates_by_index_spacing(candidates, min_spacing_vertices):
    """Keep the strongest nearby corner candidate on open chains."""

    if not candidates:
        return []

    filtered = [candidates[0]]
    for candidate in candidates[1:]:
        prev_candidate = filtered[-1]
        if candidate.index - prev_candidate.index < min_spacing_vertices:
            if candidate.turn_angle_deg > prev_candidate.turn_angle_deg:
                filtered[-1] = candidate
            continue
        filtered.append(candidate)

    return filtered


def _filter_closed_corner_candidates_by_loop_spacing(loop_vert_cos, candidates, min_span_length, min_vertex_gap):
    """Cluster closed-loop corner candidates by wrap-aware arc spacing."""

    if not candidates:
        return []

    vertex_count = len(loop_vert_cos)
    filtered: list[_CornerTurnCandidate] = []
    for candidate in candidates:
        if not filtered:
            filtered.append(candidate)
            continue

        prev_candidate = filtered[-1]
        span_length = _loop_arc_length(loop_vert_cos, prev_candidate.index, candidate.index)
        span_vertex_count = (candidate.index - prev_candidate.index) % vertex_count
        if span_vertex_count < min_vertex_gap or span_length < min_span_length:
            if candidate.turn_angle_deg > prev_candidate.turn_angle_deg:
                filtered[-1] = candidate
            continue

        filtered.append(candidate)

    while len(filtered) >= 2:
        first_candidate = filtered[0]
        last_candidate = filtered[-1]
        wrap_span_length = _loop_arc_length(loop_vert_cos, last_candidate.index, first_candidate.index)
        wrap_span_vertex_count = (first_candidate.index - last_candidate.index) % vertex_count
        if wrap_span_vertex_count >= min_vertex_gap and wrap_span_length >= min_span_length:
            break

        if first_candidate.turn_angle_deg >= last_candidate.turn_angle_deg:
            filtered.pop()
        else:
            filtered.pop(0)

    return filtered


def _detect_closed_loop_corner_indices(loop_vert_cos, basis_u, basis_v, loop_vert_indices=None, bm=None):
    """Unified closed-loop corner detection used by geometric OUTER-loop split."""

    policy = _CornerDetectionPolicy(
        closed_loop=True,
        min_spacing_vertices=1,
        min_corner_count=4,
        loop_min_span_fraction=0.04,
        relaxed_loop_min_span_fraction=0.015,
        filter_all_bevel_turns=True,
        allow_raw_non_hairpin_fallback=True,
    )

    raw_candidates, candidate_corners = _collect_corner_turn_candidates_shared(
        loop_vert_cos,
        basis_u,
        basis_v,
        policy,
        vert_indices=loop_vert_indices,
        bm=bm,
    )
    if len(candidate_corners) < policy.min_corner_count and len(raw_candidates) >= policy.min_corner_count:
        non_hairpin_candidates = [
            candidate
            for candidate in raw_candidates
            if candidate.turn_angle_deg < 150.0
        ]
        if policy.allow_raw_non_hairpin_fallback and len(non_hairpin_candidates) >= policy.min_corner_count:
            print(f"[CFTUV][GeoSplit] fallback raw_non_hairpin candidates={len(non_hairpin_candidates)}")
            candidate_corners = non_hairpin_candidates

    if len(candidate_corners) < policy.min_corner_count:
        return []

    perimeter = _loop_arc_length(loop_vert_cos, 0, 0)
    strict_min_span_length = max(perimeter * policy.loop_min_span_fraction, 1e-4)
    filtered_corners = _filter_closed_corner_candidates_by_loop_spacing(
        loop_vert_cos,
        candidate_corners,
        strict_min_span_length,
        policy.min_spacing_vertices,
    )
    if (
        len(filtered_corners) < policy.min_corner_count
        and len(candidate_corners) >= policy.min_corner_count
        and policy.relaxed_loop_min_span_fraction > 0.0
    ):
        relaxed_min_span_length = max(perimeter * policy.relaxed_loop_min_span_fraction, 1e-4)
        if relaxed_min_span_length < strict_min_span_length:
            relaxed_corners = _filter_closed_corner_candidates_by_loop_spacing(
                loop_vert_cos,
                candidate_corners,
                relaxed_min_span_length,
                policy.min_spacing_vertices,
            )
            if len(relaxed_corners) >= policy.min_corner_count:
                filtered_corners = relaxed_corners

    if len(filtered_corners) < policy.min_corner_count:
        return []

    return [candidate.index for candidate in filtered_corners]


def _detect_open_border_corner_indices(vert_cos, basis_u, basis_v, min_spacing=2,
                                       vert_indices=None, bm=None):
    """Unified open-border corner detection used before border-chain split."""

    policy = _CornerDetectionPolicy(
        closed_loop=False,
        min_spacing_vertices=min_spacing,
        reject_projected_hairpins=True,
        filter_hairpin_bevel_turns=True,
    )

    _, candidates = _collect_corner_turn_candidates_shared(
        vert_cos,
        basis_u,
        basis_v,
        policy,
        vert_indices=vert_indices,
        bm=bm,
    )
    if not candidates:
        return []

    filtered = _filter_corner_candidates_by_index_spacing(candidates, policy.min_spacing_vertices)
    return [candidate.index for candidate in filtered]


def _detect_open_border_corners(raw_chain, basis_u, basis_v, bm=None, min_spacing=2):
    """Run the full open-border corner pipeline and return staged detection results."""

    candidate_indices = _detect_open_border_corner_indices(
        raw_chain.vert_cos,
        basis_u,
        basis_v,
        min_spacing=min_spacing,
        vert_indices=raw_chain.vert_indices,
        bm=bm,
    )
    supported_indices = _filter_open_chain_corner_indices_with_support(
        raw_chain,
        candidate_indices,
        basis_u,
        basis_v,
    )
    return _OpenBorderCornerDetectionResult(
        candidate_indices=tuple(candidate_indices),
        supported_indices=tuple(supported_indices),
    )


def _measure_polyline_length(vert_cos):
    """Measure total polyline length for local split/support diagnostics."""

    if len(vert_cos) < 2:
        return 0.0

    total_length = 0.0
    for point_index in range(len(vert_cos) - 1):
        total_length += (vert_cos[point_index + 1] - vert_cos[point_index]).length
    return total_length


def _measure_open_corner_side_support(vert_cos, corner_index, basis_u, basis_v, direction, target_length):
    """Measure directional support around one open-border corner candidate.

    direction = -1 measures the incoming side toward the corner.
    direction = +1 measures the outgoing side from the corner.
    """

    projected_segments = []
    support_length = 0.0

    if direction < 0:
        segment_indices = range(corner_index - 1, -1, -1)
    else:
        segment_indices = range(corner_index, len(vert_cos) - 1)

    for segment_index in segment_indices:
        delta_3d = vert_cos[segment_index + 1] - vert_cos[segment_index]
        seg_len = delta_3d.length
        if seg_len < 1e-8:
            continue

        delta_2d = Vector((delta_3d.dot(basis_u), delta_3d.dot(basis_v)))
        if delta_2d.length_squared < 1e-12:
            continue

        projected_segments.append(delta_2d)
        support_length += seg_len
        if support_length >= target_length:
            break

    if not projected_segments or support_length < 1e-8:
        return None

    support_vec = Vector((0.0, 0.0))
    for segment in projected_segments:
        support_vec += segment
    if support_vec.length_squared < 1e-12:
        return None

    support_dir = support_vec.normalized()
    min_alignment = 1.0
    for segment in projected_segments:
        segment_dir = segment.normalized()
        min_alignment = min(min_alignment, support_dir.dot(segment_dir))

    return {
        "support_length": support_length,
        "direction": support_dir,
        "min_alignment": min_alignment,
        "segment_count": len(projected_segments),
    }


def _open_corner_has_support(vert_cos, corner_index, basis_u, basis_v):
    """Require a candidate to have stable geometric support on both sides."""

    chain_length = _measure_polyline_length(vert_cos)
    if chain_length < 1e-8:
        return None

    min_support_length = max(0.03, min(chain_length * 0.05, 0.35))
    min_alignment = 0.90

    prev_support = _measure_open_corner_side_support(
        vert_cos, corner_index, basis_u, basis_v, direction=-1, target_length=min_support_length
    )
    next_support = _measure_open_corner_side_support(
        vert_cos, corner_index, basis_u, basis_v, direction=+1, target_length=min_support_length
    )

    if prev_support is None or next_support is None:
        return None

    if (
        prev_support["support_length"] < min_support_length
        or next_support["support_length"] < min_support_length
    ):
        return None

    if (
        prev_support["min_alignment"] < min_alignment
        or next_support["min_alignment"] < min_alignment
    ):
        return None

    support_turn = math.degrees(prev_support["direction"].angle(next_support["direction"], 0.0))
    if support_turn < CORNER_ANGLE_THRESHOLD_DEG:
        return None

    return {
        "support_turn": support_turn,
        "min_support_length": min_support_length,
        "prev_support": prev_support,
        "next_support": next_support,
    }


def _measure_open_segment_direction(vert_cos, start_index, end_index, basis_u, basis_v):
    """Measure aggregate 2D direction for a contiguous open-chain segment."""

    if end_index <= start_index:
        return None

    support_vec = Vector((0.0, 0.0))
    total_length = 0.0
    for point_index in range(start_index, end_index):
        delta_3d = vert_cos[point_index + 1] - vert_cos[point_index]
        seg_len = delta_3d.length
        if seg_len < 1e-8:
            continue

        delta_2d = Vector((delta_3d.dot(basis_u), delta_3d.dot(basis_v)))
        if delta_2d.length_squared < 1e-12:
            continue
        support_vec += delta_2d
        total_length += seg_len

    if support_vec.length_squared < 1e-12 or total_length < 1e-8:
        return None

    return {
        "direction": support_vec.normalized(),
        "length": total_length,
    }


def _open_corner_pair_has_support(vert_cos, left_corner_index, right_corner_index, basis_u, basis_v):
    """Accept close paired corners that form a short but legitimate connector."""

    if right_corner_index <= left_corner_index:
        return None

    chain_length = _measure_polyline_length(vert_cos)
    if chain_length < 1e-8:
        return None

    middle_direction = _measure_open_segment_direction(
        vert_cos, left_corner_index, right_corner_index, basis_u, basis_v
    )
    if middle_direction is None:
        return None

    middle_points = [point.copy() for point in vert_cos[left_corner_index:right_corner_index + 1]]
    middle_metrics = _measure_chain_axis_metrics(middle_points, basis_u, basis_v)
    if middle_metrics is None:
        return None

    connector_length = middle_metrics["total_length"]
    connector_length_limit = max(0.04, min(chain_length * 0.05, 0.25))
    if connector_length > connector_length_limit:
        return None

    outer_support_target = max(0.03, connector_length * 2.0)
    left_outer = _measure_open_corner_side_support(
        vert_cos, left_corner_index, basis_u, basis_v, direction=-1, target_length=outer_support_target
    )
    right_outer = _measure_open_corner_side_support(
        vert_cos, right_corner_index, basis_u, basis_v, direction=+1, target_length=outer_support_target
    )
    if left_outer is None or right_outer is None:
        return None

    if left_outer["min_alignment"] < 0.90 or right_outer["min_alignment"] < 0.90:
        return None

    left_turn = math.degrees(left_outer["direction"].angle(middle_direction["direction"], 0.0))
    right_turn = math.degrees(middle_direction["direction"].angle(right_outer["direction"], 0.0))
    if left_turn < CORNER_ANGLE_THRESHOLD_DEG or right_turn < CORNER_ANGLE_THRESHOLD_DEG:
        return None

    middle_role = _classify_chain_frame_role(middle_points, basis_u, basis_v, strict_guards=False)
    if middle_role == FrameRole.FREE:
        return None

    return {
        "connector_length": connector_length,
        "middle_role": middle_role,
        "left_turn": left_turn,
        "right_turn": right_turn,
    }


def _filter_open_chain_corner_indices_with_support(raw_chain, corner_indices, basis_u, basis_v):
    """Keep only hard open-border corners that create meaningful split segments.

    This is still an analysis-stage split filter: it does not mutate roles and does
    not rescue topology after the fact. It only rejects weak corner candidates and
    obvious same-role micro-gap splits.
    """

    if not corner_indices:
        return []

    vert_cos = raw_chain.vert_cos
    supported_corner_indices = []
    for corner_index in corner_indices:
        support = _open_corner_has_support(vert_cos, corner_index, basis_u, basis_v)
        if support is None:
            print(f"[CFTUV][CornerDetect] skip idx={corner_index} reason=insufficient_support")
            continue

        print(
            f"[CFTUV][CornerDetect] keep idx={corner_index} support_turn={support['support_turn']:.1f} "
            f"support_prev={support['prev_support']['support_length']:.4f} "
            f"support_next={support['next_support']['support_length']:.4f} "
            f"align_prev={support['prev_support']['min_alignment']:.3f} "
            f"align_next={support['next_support']['min_alignment']:.3f}"
        )
        supported_corner_indices.append(corner_index)

    if len(supported_corner_indices) < 2:
        supported_corner_set = set(supported_corner_indices)
    else:
        supported_corner_set = set(supported_corner_indices)

    for corner_pos in range(len(corner_indices) - 1):
        left_corner_index = corner_indices[corner_pos]
        right_corner_index = corner_indices[corner_pos + 1]
        if left_corner_index in supported_corner_set and right_corner_index in supported_corner_set:
            continue

        pair_support = _open_corner_pair_has_support(
            vert_cos, left_corner_index, right_corner_index, basis_u, basis_v
        )
        if pair_support is None:
            continue

        print(
            f"[CFTUV][CornerDetect] keep_pair idxs=[{left_corner_index}, {right_corner_index}] "
            f"connector_role={pair_support['middle_role'].value} "
            f"connector_length={pair_support['connector_length']:.4f} "
            f"turns={pair_support['left_turn']:.1f}/{pair_support['right_turn']:.1f}"
        )
        supported_corner_set.add(left_corner_index)
        supported_corner_set.add(right_corner_index)

    supported_corner_indices = sorted(supported_corner_set)
    if len(supported_corner_indices) < 2:
        return supported_corner_indices

    chain_length = _measure_polyline_length(vert_cos)
    micro_gap_length_limit = max(0.04, min(chain_length * 0.05, 0.20))

    filtered_corner_indices = list(supported_corner_indices)
    changed = True
    while changed and len(filtered_corner_indices) >= 2:
        changed = False
        sub_chains = _split_open_chain_at_corners(raw_chain, filtered_corner_indices)
        if len(sub_chains) < 3:
            break

        segment_lengths = [_measure_polyline_length(sub_chain.vert_cos) for sub_chain in sub_chains]
        segment_roles = [
            _classify_chain_frame_role(sub_chain.vert_cos, basis_u, basis_v, strict_guards=False)
            for sub_chain in sub_chains
        ]

        for segment_index in range(1, len(sub_chains) - 1):
            segment_length = segment_lengths[segment_index]
            if segment_length > micro_gap_length_limit:
                continue

            prev_length = segment_lengths[segment_index - 1]
            next_length = segment_lengths[segment_index + 1]
            prev_role = segment_roles[segment_index - 1]
            curr_role = segment_roles[segment_index]
            next_role = segment_roles[segment_index + 1]

            has_supported_neighbors = (
                prev_length >= segment_length * 2.0 and next_length >= segment_length * 2.0
            )
            same_role_bridge = (
                prev_role == next_role
                and prev_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
                and curr_role in (FrameRole.FREE, prev_role)
            )
            if not (has_supported_neighbors and same_role_bridge):
                continue

            left_corner = filtered_corner_indices[segment_index - 1]
            right_corner = filtered_corner_indices[segment_index]
            print(
                f"[CFTUV][BorderSplit] drop micro_gap corners=[{left_corner}, {right_corner}] "
                f"role={curr_role.value} bridge={prev_role.value}->{next_role.value} "
                f"length={segment_length:.4f} prev={prev_length:.4f} next={next_length:.4f}"
            )
            del filtered_corner_indices[segment_index - 1:segment_index + 1]
            changed = True
            break

    return filtered_corner_indices


def _split_open_chain_at_corners(raw_chain, corner_indices):
    """Split one open raw chain at interior corner indices into multiple sub-chains.

    Each corner vertex becomes the end of one sub-chain and the start of the next.
    """
    vert_indices = raw_chain.vert_indices
    vert_cos = raw_chain.vert_cos
    edge_indices = raw_chain.edge_indices
    side_face_indices = raw_chain.side_face_indices
    neighbor = raw_chain.neighbor

    # Границы сегментов: [0, corner1, corner2, ..., len-1]
    boundaries = [0] + sorted(corner_indices) + [len(vert_indices) - 1]

    parent_start_loop_index = raw_chain.start_loop_index

    sub_chains = []
    for seg_idx in range(len(boundaries) - 1):
        start = boundaries[seg_idx]
        end = boundaries[seg_idx + 1]
        if end <= start:
            continue

        seg_verts = vert_indices[start:end + 1]
        seg_cos = [co.copy() for co in vert_cos[start:end + 1]]
        seg_edges = edge_indices[start:end] if start < len(edge_indices) else []
        seg_sides = side_face_indices[start:end + 1] if start < len(side_face_indices) else []

        if len(seg_verts) < 2:
            continue

        sub_chains.append(_RawBoundaryChain(
            vert_indices=seg_verts,
            vert_cos=seg_cos,
            edge_indices=seg_edges,
            side_face_indices=seg_sides,
            neighbor=neighbor,
            is_closed=False,
            start_loop_index=parent_start_loop_index + start,
            end_loop_index=parent_start_loop_index + end,
            is_corner_split=True,
        ))

    return sub_chains if sub_chains else [raw_chain]


def _split_border_chains_by_corners(raw_chains, basis_u, basis_v, bm=None):
    """Split all open MESH_BORDER chains at geometric corners.

    Безусловно применяется ко всем MESH_BORDER chains — не проверяем
    classification заранее. Corner detection + split, потом классификация
    каждого полученного сегмента по ratio.
    """
    result = []
    for raw_chain in raw_chains:
        neighbor = int(raw_chain.neighbor)
        is_closed = bool(raw_chain.is_closed)

        if neighbor != NB_MESH_BORDER or is_closed:
            result.append(raw_chain)
            continue

        vert_cos = raw_chain.vert_cos
        if len(vert_cos) < 3:
            result.append(raw_chain)
            continue

        # Для hairpin-like open-border candidates используем дополнительную
        # 3D/normal verification, чтобы smooth tube rims не split-ились ложно.
        detection = _detect_open_border_corners(
            raw_chain,
            basis_u,
            basis_v,
            bm=bm,
            min_spacing=1,
        )

        # Debug: показать что corner detection нашёл
        corner_indices = list(detection.supported_indices)

        points_2d = [Vector((co.dot(basis_u), co.dot(basis_v))) for co in vert_cos]
        print(
            f"[CFTUV][BorderSplit] MESH_BORDER chain verts={len(vert_cos)} "
            f"candidates={list(detection.candidate_indices)} corners={corner_indices} "
            f"pts={[(round(p.x,3), round(p.y,3)) for p in points_2d]}"
        )

        if not corner_indices:
            result.append(raw_chain)
            continue

        sub_chains = _split_open_chain_at_corners(raw_chain, corner_indices)
        print(f"[CFTUV][BorderSplit] Split into {len(sub_chains)} sub-chains")
        result.extend(sub_chains)

    return result


def _try_geometric_outer_loop_split(raw_loop, raw_chains, basis_u, basis_v, bm=None):
    """Fallback split for isolated OUTER loops that collapsed into one FREE chain."""

    loop_kind = raw_loop.kind
    if not isinstance(loop_kind, LoopKind):
        loop_kind = LoopKind(loop_kind)

    if loop_kind != LoopKind.OUTER or len(raw_chains) != 1:
        return raw_chains

    raw_chain = raw_chains[0]
    # Любой single closed chain на OUTER loop нужно пробовать split.
    # Ранее проверялся только FREE, но прямая стена (H_FRAME/V_FRAME)
    # тоже может быть single chain если все edges имеют одного соседа.
    if not raw_chain.is_closed:
        return raw_chains

    lvi = raw_loop.vert_indices
    split_indices = _detect_closed_loop_corner_indices(
        raw_loop.vert_cos, basis_u, basis_v,
        loop_vert_indices=lvi,
        bm=bm,
    )
    print(f"[CFTUV][GeoSplit] split_indices={len(split_indices)} indices={split_indices}")
    if len(split_indices) < 4:
        print(f"[CFTUV][GeoSplit] BAIL: <4 split indices")
        return raw_chains

    derived_raw_chains = _split_closed_loop_by_corner_indices(
        raw_loop,
        split_indices,
        int(raw_chain.neighbor),
    )
    print(f"[CFTUV][GeoSplit] derived_chains={len(derived_raw_chains)}")
    if len(derived_raw_chains) < 4:
        print(f"[CFTUV][GeoSplit] BAIL: <4 derived chains")
        return raw_chains

    derived_roles = [
        _classify_chain_frame_role(derived_raw_chain.vert_cos, basis_u, basis_v, strict_guards=False)
        for derived_raw_chain in derived_raw_chains
    ]
    print(f"[CFTUV][GeoSplit] derived_roles={[r.value for r in derived_roles]}")
    # Geometric split валиден если нашли ≥4 corner и получили ≥4 chains.
    # Не требуем наличия обоих H_FRAME и V_FRAME — patch с бевелями
    # или наклонной стеной может иметь chains которые классифицируются
    # как FREE из-за waviness/curvature, но split всё равно правильный.
    # Минимум: хотя бы 1 chain не FREE (иначе split бесполезен).
    non_free_count = sum(1 for role in derived_roles if role != FrameRole.FREE)
    if non_free_count < 1:
        print(f"[CFTUV][GeoSplit] BAIL: all derived chains are FREE")
        return raw_chains
    print(f"[CFTUV][GeoSplit] SUCCESS: split into {len(derived_raw_chains)} chains, non_free={non_free_count}")

    return derived_raw_chains


def _build_boundary_chain_objects(raw_chains, basis_u, basis_v):
    """Instantiate BoundaryChain dataclasses from raw chain payloads."""

    chains = []
    for raw_chain in raw_chains:
        chain_vert_cos = [co.copy() for co in raw_chain.vert_cos]
        neighbor_id = int(raw_chain.neighbor)
        # MESH_BORDER chains: только ratio check (strict_guards=False)
        # PATCH/SEAM chains: ratio + ослабленные waviness/curvature guards
        use_strict_guards = (neighbor_id != NB_MESH_BORDER)
        chains.append(
            BoundaryChain(
                vert_indices=list(raw_chain.vert_indices),
                vert_cos=chain_vert_cos,
                edge_indices=list(raw_chain.edge_indices),
                side_face_indices=list(raw_chain.side_face_indices),
                neighbor_patch_id=neighbor_id,
                is_closed=bool(raw_chain.is_closed),
                frame_role=_classify_chain_frame_role(chain_vert_cos, basis_u, basis_v, strict_guards=use_strict_guards),
                start_loop_index=int(raw_chain.start_loop_index),
                end_loop_index=int(raw_chain.end_loop_index),
                is_corner_split=bool(raw_chain.is_corner_split),
            )
        )
    return chains


def _measure_chain_frame_confidence(chain, basis_u, basis_v):
    """Measure chain confidence using the current asymmetric H/V semantics."""

    metrics = _measure_chain_axis_metrics(chain.vert_cos, basis_u, basis_v)
    if metrics is None:
        return _ChainFrameConfidence(
            primary_support=float("-inf"),
            total_length=0.0,
            avg_deviation_score=0.0,
            max_deviation_score=0.0,
            vert_count=0,
            edge_count=0,
        )

    if chain.frame_role == FrameRole.H_FRAME:
        primary_support = metrics["h_support"]
        avg_deviation = metrics["h_avg_deviation"]
        max_deviation = metrics["h_max_deviation"]
    elif chain.frame_role == FrameRole.V_FRAME:
        primary_support = metrics["v_support"]
        avg_deviation = metrics["v_avg_deviation"]
        max_deviation = metrics["v_max_deviation"]
    else:
        return _ChainFrameConfidence(
            primary_support=float("-inf"),
            total_length=0.0,
            avg_deviation_score=0.0,
            max_deviation_score=0.0,
            vert_count=len(chain.vert_indices),
            edge_count=len(chain.edge_indices),
        )

    return _ChainFrameConfidence(
        primary_support=primary_support,
        total_length=metrics["total_length"],
        avg_deviation_score=-avg_deviation,
        max_deviation_score=-max_deviation,
        vert_count=len(chain.vert_indices),
        edge_count=len(chain.edge_indices),
    )


@dataclass
class _FrameRun:
    """Diagnostic-only continuity view over final neighboring chains of one loop."""

    patch_id: int
    loop_index: int
    chain_indices: tuple[int, ...] = ()
    dominant_role: FrameRole = FrameRole.FREE
    start_corner_index: int = -1
    end_corner_index: int = -1
    total_length: float = 0.0
    support_length: float = 0.0
    gap_free_length: float = 0.0
    max_free_gap_length: float = 0.0
    projected_u_span: float = 0.0
    projected_v_span: float = 0.0


def _frame_run_chain_length(chain, basis_u, basis_v):
    metrics = _measure_chain_axis_metrics(chain.vert_cos, basis_u, basis_v)
    if metrics is not None:
        return metrics["total_length"]

    total = 0.0
    for point_index in range(len(chain.vert_cos) - 1):
        total += (chain.vert_cos[point_index + 1] - chain.vert_cos[point_index]).length
    return total


def _measure_frame_run_projected_span(points, basis_u, basis_v):
    if not points:
        return _ProjectedSpan2D(u_span=0.0, v_span=0.0)

    u_values = [point.dot(basis_u) for point in points]
    v_values = [point.dot(basis_v) for point in points]
    return _ProjectedSpan2D(
        u_span=max(u_values) - min(u_values),
        v_span=max(v_values) - min(v_values),
    )


def _measure_frame_run_chord_deviation(chain, basis_u, basis_v):
    if len(chain.vert_cos) < 2:
        return None

    basis_n = basis_u.cross(basis_v)
    if basis_n.length_squared < 1e-8:
        return None
    basis_n.normalize()

    chord = chain.vert_cos[-1] - chain.vert_cos[0]
    chord_length = chord.length
    if chord_length < 1e-6:
        return None

    dir_u = chord.dot(basis_u) / chord_length
    dir_v = chord.dot(basis_v) / chord_length
    dir_n = chord.dot(basis_n) / chord_length
    transverse = math.sqrt(max(0.0, dir_u * dir_u + dir_n * dir_n))
    return {
        "h_deviation": abs(dir_v),
        "v_deviation": transverse,
    }


def _infer_frame_run_gap_role(chains, chain_index, basis_u, basis_v):
    """Infer only obvious same-role micro-gaps for diagnostics.

    This does not mutate chain roles and intentionally stays conservative.
    """

    chain_count = len(chains)
    if chain_count < 3:
        return None

    chain = chains[chain_index]
    if chain.frame_role != FrameRole.FREE:
        return None
    if chain.neighbor_kind != ChainNeighborKind.MESH_BORDER or chain.is_closed:
        return None

    prev_chain = chains[(chain_index - 1) % chain_count]
    next_chain = chains[(chain_index + 1) % chain_count]
    if prev_chain.frame_role != next_chain.frame_role:
        return None
    if prev_chain.frame_role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        return None

    chain_metrics = _measure_chain_axis_metrics(chain.vert_cos, basis_u, basis_v)
    prev_length = _frame_run_chain_length(prev_chain, basis_u, basis_v)
    next_length = _frame_run_chain_length(next_chain, basis_u, basis_v)
    chord_metrics = _measure_frame_run_chord_deviation(chain, basis_u, basis_v)
    if chain_metrics is None or chord_metrics is None:
        return None

    micro_gap_limit = max(0.10, min(prev_length, next_length) * 0.20)
    if chain_metrics["total_length"] > micro_gap_limit:
        return None

    carried_role = prev_chain.frame_role
    if carried_role == FrameRole.H_FRAME:
        if (
            chord_metrics["h_deviation"] < 0.035
            and chain_metrics["h_support"] > 1e-6
            and chain_metrics["h_avg_deviation"] < 0.10
        ):
            return FrameRole.H_FRAME
        return None

    if carried_role == FrameRole.V_FRAME:
        if (
            chord_metrics["v_deviation"] < 0.08
            and chain_metrics["v_support"] > 1e-6
            and chain_metrics["v_avg_deviation"] < 0.12
        ):
            return FrameRole.V_FRAME
        return None

    return None


def _report_frame_run_invariant_violation(patch_id, loop_index, rule_code, detail):
    print(
        f"[CFTUV][TopologyInvariant] Patch {patch_id} Loop {loop_index} {rule_code} {detail}"
    )


def _derive_effective_frame_roles(chains, basis_u, basis_v):
    """Derive conservative continuity roles used by frame-run diagnostics."""

    effective_roles = []
    for chain_index, chain in enumerate(chains):
        inferred_gap_role = _infer_frame_run_gap_role(chains, chain_index, basis_u, basis_v)
        effective_roles.append(inferred_gap_role if inferred_gap_role is not None else chain.frame_role)
    return tuple(effective_roles)


def _derive_frame_run_entries(chains, effective_roles):
    """Partition loop chains into cyclic continuity runs by effective role."""

    if not chains:
        return []

    run_entries: list[_FrameRunBuildEntry] = []
    current_indices = [0]
    current_role = effective_roles[0]

    for chain_index in range(1, len(chains)):
        if effective_roles[chain_index] == current_role:
            current_indices.append(chain_index)
            continue
        run_entries.append(
            _FrameRunBuildEntry(
                dominant_role=current_role,
                chain_indices=tuple(current_indices),
            )
        )
        current_indices = [chain_index]
        current_role = effective_roles[chain_index]

    run_entries.append(
        _FrameRunBuildEntry(
            dominant_role=current_role,
            chain_indices=tuple(current_indices),
        )
    )

    if (
        len(run_entries) > 1
        and run_entries[0].dominant_role == run_entries[-1].dominant_role
    ):
        merged_role = run_entries[-1].dominant_role
        merged_indices = run_entries[-1].chain_indices + run_entries[0].chain_indices
        run_entries = [
            _FrameRunBuildEntry(
                dominant_role=merged_role,
                chain_indices=merged_indices,
            )
        ] + run_entries[1:-1]

    return run_entries


def _build_frame_run(chains, effective_roles, basis_u, basis_v, patch_id, loop_index, chain_indices):
    if not chain_indices:
        return None

    dominant_role = effective_roles[chain_indices[0]]
    run_points = []
    total_length = 0.0
    support_length = 0.0
    gap_free_length = 0.0
    max_free_gap_length = 0.0

    for local_index, chain_index in enumerate(chain_indices):
        chain = chains[chain_index]
        chain_length = _frame_run_chain_length(chain, basis_u, basis_v)
        total_length += chain_length

        if chain.frame_role == dominant_role:
            support_length += chain_length
        elif dominant_role != FrameRole.FREE and chain.frame_role == FrameRole.FREE:
            gap_free_length += chain_length
            max_free_gap_length = max(max_free_gap_length, chain_length)
        elif dominant_role == FrameRole.FREE:
            support_length += chain_length

        chain_points = [point.copy() for point in chain.vert_cos]
        if not chain_points:
            continue
        if local_index == 0:
            run_points.extend(chain_points)
            continue
        if run_points and (run_points[-1] - chain_points[0]).length_squared <= 1e-12:
            run_points.extend(chain_points[1:])
        else:
            run_points.extend(chain_points)

    projected_span = _measure_frame_run_projected_span(run_points, basis_u, basis_v)
    first_chain = chains[chain_indices[0]]
    last_chain = chains[chain_indices[-1]]
    return _FrameRun(
        patch_id=patch_id,
        loop_index=loop_index,
        chain_indices=tuple(chain_indices),
        dominant_role=dominant_role,
        start_corner_index=first_chain.start_corner_index,
        end_corner_index=last_chain.end_corner_index,
        total_length=total_length,
        support_length=support_length,
        gap_free_length=gap_free_length,
        max_free_gap_length=max_free_gap_length,
        projected_u_span=projected_span.u_span,
        projected_v_span=projected_span.v_span,
    )


def _validate_loop_frame_runs(boundary_loop, frame_runs, effective_roles, patch_id, loop_index):
    """Validate frame-run continuity against final loop chain topology."""

    chains = list(boundary_loop.chains)
    if not chains:
        if frame_runs:
            _report_frame_run_invariant_violation(
                patch_id,
                loop_index,
                "FR1",
                f"unexpected_runs_for_empty_loop count={len(frame_runs)}",
            )
        return frame_runs

    seen_chain_indices: list[int] = []
    chain_count = len(chains)
    for run_index, frame_run in enumerate(frame_runs):
        if not frame_run.chain_indices:
            _report_frame_run_invariant_violation(
                patch_id,
                loop_index,
                "FR1",
                f"empty_run run={run_index}",
            )
            continue

        first_chain = chains[frame_run.chain_indices[0]]
        last_chain = chains[frame_run.chain_indices[-1]]
        if frame_run.start_corner_index != first_chain.start_corner_index:
            _report_frame_run_invariant_violation(
                patch_id,
                loop_index,
                "FR3",
                f"start_corner_mismatch run={run_index} expected={first_chain.start_corner_index} actual={frame_run.start_corner_index}",
            )
        if frame_run.end_corner_index != last_chain.end_corner_index:
            _report_frame_run_invariant_violation(
                patch_id,
                loop_index,
                "FR3",
                f"end_corner_mismatch run={run_index} expected={last_chain.end_corner_index} actual={frame_run.end_corner_index}",
            )

        for local_index, chain_index in enumerate(frame_run.chain_indices):
            seen_chain_indices.append(chain_index)
            if chain_index < 0 or chain_index >= chain_count:
                _report_frame_run_invariant_violation(
                    patch_id,
                    loop_index,
                    "FR1",
                    f"chain_index_out_of_range run={run_index} chain={chain_index} count={chain_count}",
                )
                continue

            expected_role = effective_roles[chain_index]
            if expected_role != frame_run.dominant_role:
                _report_frame_run_invariant_violation(
                    patch_id,
                    loop_index,
                    "FR2",
                    f"role_mismatch run={run_index} chain={chain_index} expected={expected_role.value} actual={frame_run.dominant_role.value}",
                )

            if local_index == 0:
                continue
            prev_chain_index = frame_run.chain_indices[local_index - 1]
            expected_chain_index = (prev_chain_index + 1) % chain_count
            if chain_index != expected_chain_index:
                _report_frame_run_invariant_violation(
                    patch_id,
                    loop_index,
                    "FR4",
                    f"non_contiguous_run run={run_index} prev={prev_chain_index} actual={chain_index} expected={expected_chain_index}",
                )

    expected_coverage = list(range(chain_count))
    actual_coverage = sorted(seen_chain_indices)
    if actual_coverage != expected_coverage:
        _report_frame_run_invariant_violation(
            patch_id,
            loop_index,
            "FR1",
            f"coverage_mismatch expected={expected_coverage} actual={actual_coverage}",
        )

    if len(seen_chain_indices) != len(set(seen_chain_indices)):
        _report_frame_run_invariant_violation(
            patch_id,
            loop_index,
            "FR1",
            f"duplicate_chain_coverage seen={seen_chain_indices}",
        )

    return frame_runs


def _build_loop_frame_run_result(boundary_loop, basis_u, basis_v, patch_id, loop_index):
    """Build and validate diagnostic continuity runs over final chains of one loop."""

    chains = list(boundary_loop.chains)
    if not chains:
        return _LoopFrameRunBuildResult(effective_roles=(), runs=())

    effective_roles = _derive_effective_frame_roles(chains, basis_u, basis_v)
    run_entries = _derive_frame_run_entries(chains, effective_roles)

    frame_runs = []
    for run_entry in run_entries:
        frame_run = _build_frame_run(
            chains,
            effective_roles,
            basis_u,
            basis_v,
            patch_id,
            loop_index,
            run_entry.chain_indices,
        )
        if frame_run is not None:
            frame_runs.append(frame_run)
    _validate_loop_frame_runs(boundary_loop, frame_runs, effective_roles, patch_id, loop_index)
    return _LoopFrameRunBuildResult(effective_roles=effective_roles, runs=tuple(frame_runs))


def _build_patch_graph_loop_frame_results(graph):
    """Build per-loop derived frame-run results for the PatchGraph."""

    loop_frame_results: dict[PatchLoopKey, _LoopFrameRunBuildResult] = {}
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            build_result = _build_loop_frame_run_result(
                boundary_loop,
                node.basis_u,
                node.basis_v,
                patch_id,
                loop_index,
            )
            loop_frame_results[(patch_id, loop_index)] = build_result
    return loop_frame_results


def _downgrade_same_role_point_contact_chains(chains, basis_u, basis_v, patch_id, loop_index):
    """Downgrade weaker same-role chains when they touch only at one vertex."""

    chain_count = len(chains)
    if chain_count < 2:
        return

    pair_indices = [(chain_index, chain_index + 1) for chain_index in range(chain_count - 1)]
    if chain_count > 2:
        pair_indices.append((chain_count - 1, 0))

    for first_index, second_index in pair_indices:
        first_chain = chains[first_index]
        second_chain = chains[second_index]
        if first_chain.frame_role != second_chain.frame_role:
            continue
        if first_chain.frame_role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            continue
        if (
            first_chain.neighbor_kind == ChainNeighborKind.MESH_BORDER
            and second_chain.neighbor_kind == ChainNeighborKind.MESH_BORDER
        ):
            # Same-patch border H/V pieces should merge later, not fight via point-contact downgrade.
            continue

        shared_vert_indices = (set(first_chain.vert_indices) & set(second_chain.vert_indices)) - {-1}
        if len(shared_vert_indices) != 1:
            continue

        first_strength = _measure_chain_frame_confidence(first_chain, basis_u, basis_v)
        second_strength = _measure_chain_frame_confidence(second_chain, basis_u, basis_v)
        weaker_index = second_index if first_strength >= second_strength else first_index
        stronger_index = first_index if weaker_index == second_index else second_index
        weaker_chain = chains[weaker_index]
        if weaker_chain.frame_role == FrameRole.FREE:
            continue

        shared_vert_index = next(iter(shared_vert_indices))
        print(
            f"[CFTUV][RoleConflict] Patch {patch_id} Loop {loop_index} "
            f"{first_chain.frame_role.value} point_contact vert={shared_vert_index} "
            f"keep=C{stronger_index} drop=C{weaker_index}->FREE"
        )
        weaker_chain.frame_role = FrameRole.FREE

def _merge_same_role_border_chains(chains):
    """Сшивает adjacent MESH_BORDER chains с одинаковым frame_role.

    После GeoSplit и corner split могут возникнуть маленькие H/V sub-chains
    на бевельных гранях. Если два соседних chain — оба MESH_BORDER, оба
    одного role (H+H, V+V) — сшиваем в один. Бевельные вершины становятся
    частью длинного chain, direction считается один раз корректно.
    """
    if len(chains) < 2:
        return chains

    merged = [chains[0]]
    for i in range(1, len(chains)):
        prev = merged[-1]
        curr = chains[i]

        can_merge = (
            prev.neighbor_kind == ChainNeighborKind.MESH_BORDER
            and curr.neighbor_kind == ChainNeighborKind.MESH_BORDER
            and prev.frame_role == curr.frame_role
            and prev.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
            and not prev.is_closed
            and not curr.is_closed
        )

        if can_merge:
            # Сшиваем: prev поглощает curr (shared vertex = prev[-1] == curr[0])
            merged[-1] = BoundaryChain(
                vert_indices=prev.vert_indices + curr.vert_indices[1:],
                vert_cos=prev.vert_cos + curr.vert_cos[1:],
                edge_indices=prev.edge_indices + curr.edge_indices,
                side_face_indices=prev.side_face_indices + curr.side_face_indices[1:],
                neighbor_patch_id=prev.neighbor_patch_id,
                is_closed=False,
                frame_role=prev.frame_role,
                start_loop_index=prev.start_loop_index,
                end_loop_index=curr.end_loop_index,
                is_corner_split=prev.is_corner_split and curr.is_corner_split,
            )
        else:
            merged.append(curr)

    # Проверяем wrap-around: последний и первый chain в closed loop
    if len(merged) >= 2:
        last = merged[-1]
        first = merged[0]
        can_wrap = (
            last.neighbor_kind == ChainNeighborKind.MESH_BORDER
            and first.neighbor_kind == ChainNeighborKind.MESH_BORDER
            and last.frame_role == first.frame_role
            and last.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
            and not last.is_closed
            and not first.is_closed
        )
        if can_wrap:
            merged[0] = BoundaryChain(
                vert_indices=last.vert_indices + first.vert_indices[1:],
                vert_cos=last.vert_cos + first.vert_cos[1:],
                edge_indices=last.edge_indices + first.edge_indices,
                side_face_indices=last.side_face_indices + first.side_face_indices[1:],
                neighbor_patch_id=last.neighbor_patch_id,
                is_closed=False,
                frame_role=last.frame_role,
                start_loop_index=last.start_loop_index,
                end_loop_index=first.end_loop_index,
                is_corner_split=last.is_corner_split and first.is_corner_split,
            )
            merged.pop()

    return merged


def _resolve_loop_corner_from_final_chains(boundary_loop, prev_chain, next_chain):
    """Resolve junction-corner identity directly from final chain endpoints.

    Здесь больше нет loop-wide best-match search. Если final chains уже не могут
    честно указать shared endpoint, мы не угадываем corner по всему loop, а
    только мягко деградируем к chain endpoint payload для diagnostics.
    """

    next_start_vert_index = next_chain.start_vert_index
    prev_end_vert_index = prev_chain.end_vert_index
    shared_vert_index = next_start_vert_index if next_start_vert_index >= 0 else prev_end_vert_index
    resolved_exactly = True

    if (
        next_start_vert_index >= 0
        and prev_end_vert_index >= 0
        and next_start_vert_index != prev_end_vert_index
    ):
        resolved_exactly = False
        shared_vert_index = next_start_vert_index

    vertex_count = len(boundary_loop.vert_indices)
    if vertex_count > 0 and shared_vert_index >= 0:
        loop_vert_index = next_chain.start_loop_index % vertex_count
        prev_loop_vert_index = prev_chain.end_loop_index % vertex_count
        if prev_loop_vert_index != loop_vert_index:
            resolved_exactly = False

        if (
            0 <= loop_vert_index < len(boundary_loop.vert_indices)
            and boundary_loop.vert_indices[loop_vert_index] == shared_vert_index
            and 0 <= loop_vert_index < len(boundary_loop.vert_cos)
        ):
            return _ResolvedLoopCornerIdentity(
                loop_vert_index=loop_vert_index,
                vert_index=shared_vert_index,
                vert_co=boundary_loop.vert_cos[loop_vert_index].copy(),
                resolved_exactly=resolved_exactly,
            )

        resolved_exactly = False

    if next_chain.vert_cos:
        return _ResolvedLoopCornerIdentity(
            loop_vert_index=int(next_chain.start_loop_index),
            vert_index=int(next_chain.start_vert_index),
            vert_co=next_chain.vert_cos[0].copy(),
            resolved_exactly=False,
        )
    if prev_chain.vert_cos:
        return _ResolvedLoopCornerIdentity(
            loop_vert_index=int(prev_chain.end_loop_index),
            vert_index=int(prev_chain.end_vert_index),
            vert_co=prev_chain.vert_cos[-1].copy(),
            resolved_exactly=False,
        )
    return _ResolvedLoopCornerIdentity(
        loop_vert_index=-1,
        vert_index=-1,
        vert_co=Vector((0.0, 0.0, 0.0)),
        resolved_exactly=False,
    )


def _report_boundary_loop_invariant_violation(patch_id, loop_index, rule_code, detail):
    """Emit a deterministic topology invariant violation without aborting analysis."""

    print(f"[CFTUV][TopologyInvariant] Patch {patch_id} Loop {loop_index} {rule_code} {detail}")


def _report_patch_topology_invariant_violation(patch_id, rule_code, detail):
    """Emit a deterministic patch-level topology invariant violation."""

    print(f"[CFTUV][TopologyInvariant] Patch {patch_id} {rule_code} {detail}")


def _report_graph_topology_invariant_violation(rule_code, detail):
    """Emit a deterministic graph-level topology invariant violation."""

    print(f"[CFTUV][TopologyInvariant] Graph {rule_code} {detail}")


def _report_junction_invariant_violation(vert_index, rule_code, detail):
    """Emit a deterministic junction-level topology invariant violation."""

    print(f"[CFTUV][TopologyInvariant] Junction V{vert_index} {rule_code} {detail}")


def _report_loop_classification_diagnostic(patch_id, detail):
    """Emit diagnostics for non-authoritative shadow loop classification comparisons."""

    print(f"[CFTUV][LoopClassDiag] Patch {patch_id} {detail}")


def _loop_vertex_span(loop_vert_indices, start_loop_index, end_loop_index):
    """Collect the expected closed-loop vertex span from start to end inclusive."""

    vertex_count = len(loop_vert_indices)
    if vertex_count == 0:
        return []

    start_loop_index %= vertex_count
    end_loop_index %= vertex_count
    if start_loop_index == end_loop_index:
        return list(loop_vert_indices)

    span = [loop_vert_indices[start_loop_index]]
    loop_index = start_loop_index
    safety = 0
    while safety < vertex_count:
        safety += 1
        loop_index = (loop_index + 1) % vertex_count
        span.append(loop_vert_indices[loop_index])
        if loop_index == end_loop_index:
            break
    return span


def _loop_edge_span(loop_edge_indices, start_loop_index, end_loop_index):
    """Collect the expected closed-loop edge span from start to end exclusive."""

    edge_count = len(loop_edge_indices)
    if edge_count == 0:
        return []

    start_loop_index %= edge_count
    end_loop_index %= edge_count
    if start_loop_index == end_loop_index:
        return list(loop_edge_indices)

    span = []
    loop_index = start_loop_index
    safety = 0
    while safety < edge_count:
        safety += 1
        span.append(loop_edge_indices[loop_index])
        loop_index = (loop_index + 1) % edge_count
        if loop_index == end_loop_index:
            break
    return span


def _validate_boundary_loop_topology(boundary_loop, patch_id, loop_index):
    """Validate key loop/chain/corner invariants after final topology derivation."""

    loop_vertex_count = len(boundary_loop.vert_indices)
    loop_edge_count = len(boundary_loop.edge_indices)
    chain_count = len(boundary_loop.chains)
    corner_count = len(boundary_loop.corners)

    total_chain_edges = 0
    for chain_index, chain in enumerate(boundary_loop.chains):
        expected_vert_indices = _loop_vertex_span(
            boundary_loop.vert_indices,
            chain.start_loop_index,
            chain.end_loop_index,
        )
        if chain.vert_indices != expected_vert_indices:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "X8",
                f"chain={chain_index} vert_span_mismatch expected={expected_vert_indices} actual={chain.vert_indices}",
            )

        expected_edge_indices = _loop_edge_span(
            boundary_loop.edge_indices,
            chain.start_loop_index,
            chain.end_loop_index,
        )
        if chain.edge_indices != expected_edge_indices:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "X7",
                f"chain={chain_index} edge_span_mismatch expected={expected_edge_indices} actual={chain.edge_indices}",
            )

        total_chain_edges += len(chain.edge_indices)

    if total_chain_edges != loop_edge_count:
        _report_boundary_loop_invariant_violation(
            patch_id,
            loop_index,
            "X7",
            f"edge_coverage_mismatch loop_edges={loop_edge_count} chain_edges={total_chain_edges}",
        )

    if chain_count >= 2 and corner_count != chain_count:
        _report_boundary_loop_invariant_violation(
            patch_id,
            loop_index,
            "X2",
            f"corner_count_mismatch chains={chain_count} corners={corner_count}",
        )

    if chain_count >= 2:
        for next_chain_index in range(chain_count):
            prev_chain_index = (next_chain_index - 1) % chain_count
            prev_chain = boundary_loop.chains[prev_chain_index]
            next_chain = boundary_loop.chains[next_chain_index]
            if prev_chain.end_vert_index != next_chain.start_vert_index:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "X1",
                    f"chain_endpoint_mismatch prev_chain={prev_chain_index} next_chain={next_chain_index} "
                    f"prev_end={prev_chain.end_vert_index} next_start={next_chain.start_vert_index}",
                )

        for corner_index, corner in enumerate(boundary_loop.corners):
            if corner.corner_kind != CornerKind.JUNCTION:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "R1",
                    f"corner={corner_index} expected=JUNCTION actual={corner.corner_kind.value}",
                )

            if not (0 <= corner.prev_chain_index < chain_count and 0 <= corner.next_chain_index < chain_count):
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "R7",
                    f"corner={corner_index} invalid_chain_refs prev={corner.prev_chain_index} next={corner.next_chain_index}",
                )
                continue

            prev_chain = boundary_loop.chains[corner.prev_chain_index]
            next_chain = boundary_loop.chains[corner.next_chain_index]
            if corner.vert_index != prev_chain.end_vert_index:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "X3",
                    f"corner={corner_index} vert={corner.vert_index} prev_end={prev_chain.end_vert_index}",
                )
            if corner.vert_index != next_chain.start_vert_index:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "X4",
                    f"corner={corner_index} vert={corner.vert_index} next_start={next_chain.start_vert_index}",
                )

        for chain_index, chain in enumerate(boundary_loop.chains):
            expected_start_corner = chain_index
            expected_end_corner = (chain_index + 1) % chain_count
            if chain.start_corner_index != expected_start_corner:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "R7",
                    f"chain={chain_index} start_corner_mismatch expected={expected_start_corner} actual={chain.start_corner_index}",
                )
            if chain.end_corner_index != expected_end_corner:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "R7",
                    f"chain={chain_index} end_corner_mismatch expected={expected_end_corner} actual={chain.end_corner_index}",
                )

    elif chain_count == 1:
        for corner_index, corner in enumerate(boundary_loop.corners):
            if corner.corner_kind != CornerKind.GEOMETRIC:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "R2",
                    f"corner={corner_index} expected=GEOMETRIC actual={corner.corner_kind.value}",
                )
            if corner.prev_chain_index != 0 or corner.next_chain_index != 0:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "R2",
                    f"corner={corner_index} single_chain_refs prev={corner.prev_chain_index} next={corner.next_chain_index}",
                )
    elif loop_vertex_count > 0 or loop_edge_count > 0:
        _report_boundary_loop_invariant_violation(
            patch_id,
            loop_index,
            "C1",
            "loop_has_geometry_but_no_chains",
        )


def _validate_patch_loop_classification(node):
    """Validate OUTER/HOLE classification invariants after loop build."""

    if not node.boundary_loops:
        return

    outer_loop_count = sum(1 for boundary_loop in node.boundary_loops if boundary_loop.kind == LoopKind.OUTER)
    if outer_loop_count != 1:
        _report_patch_topology_invariant_violation(
            node.patch_id,
            "L1",
            f"outer_loop_count_mismatch expected=1 actual={outer_loop_count}",
        )

    for loop_index, boundary_loop in enumerate(node.boundary_loops):
        expected_kind = LoopKind.OUTER if boundary_loop.depth % 2 == 0 else LoopKind.HOLE
        if boundary_loop.kind != expected_kind:
            _report_boundary_loop_invariant_violation(
                node.patch_id,
                loop_index,
                "L3",
                f"depth_parity_kind_mismatch depth={boundary_loop.depth} expected={expected_kind.value} "
                f"actual={boundary_loop.kind.value}",
            )


def _collect_patch_boundary_edge_indices(patch_face_indices, bm):
    """Collect the authoritative boundary-edge set for one patch."""

    patch_face_index_set = set(patch_face_indices)
    boundary_edge_indices = set()
    for face_index in patch_face_index_set:
        face = bm.faces[face_index]
        for loop in face.loops:
            if _is_boundary_side(loop, patch_face_index_set):
                boundary_edge_indices.add(loop.edge.index)
    return boundary_edge_indices


def _validate_raw_patch_boundary_topology(patch_id, patch_data, bm):
    """Validate raw loop coverage and overlap before final loop serialization."""

    raw_loops = patch_data.raw_loops
    if not raw_loops:
        return

    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    expected_boundary_edges = _collect_patch_boundary_edge_indices(patch_data.face_indices, bm)
    edge_to_loops: dict[int, set[int]] = {}
    vert_to_loops: dict[int, set[int]] = {}
    patch_face_index_set = set(patch_data.face_indices)

    for loop_index, raw_loop in enumerate(raw_loops):
        edge_count = len(raw_loop.edge_indices)
        vert_count = len(raw_loop.vert_indices)
        side_count = len(raw_loop.side_face_indices)

        if edge_count != vert_count:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "L6",
                f"raw_edge_vert_count_mismatch edges={edge_count} verts={vert_count}",
            )
        if edge_count != side_count:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "L6",
                f"raw_edge_side_count_mismatch edges={edge_count} side_faces={side_count}",
            )
        if len(set(raw_loop.edge_indices)) != edge_count:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "L6",
                "raw_loop_repeats_boundary_edge",
            )

        for side_face_index in raw_loop.side_face_indices:
            if side_face_index not in patch_face_index_set:
                _report_boundary_loop_invariant_violation(
                    patch_id,
                    loop_index,
                    "L6",
                    f"raw_side_face_out_of_patch face={side_face_index}",
                )

        for edge_index in raw_loop.edge_indices:
            edge_to_loops.setdefault(edge_index, set()).add(loop_index)

        for vert_index in set(raw_loop.vert_indices):
            vert_to_loops.setdefault(vert_index, set()).add(loop_index)

    actual_boundary_edges = set(edge_to_loops.keys())
    missing_edges = sorted(expected_boundary_edges - actual_boundary_edges)
    extra_edges = sorted(actual_boundary_edges - expected_boundary_edges)
    if missing_edges or extra_edges:
        _report_patch_topology_invariant_violation(
            patch_id,
            "L6",
            f"raw_boundary_edge_coverage_mismatch missing={missing_edges} extra={extra_edges}",
        )

    for edge_index, loop_indices in edge_to_loops.items():
        if len(loop_indices) != 1:
            _report_patch_topology_invariant_violation(
                patch_id,
                "L6",
                f"boundary_edge_multi_loop edge={edge_index} loops={sorted(loop_indices)}",
            )

    for vert_index, loop_indices in vert_to_loops.items():
        if len(loop_indices) < 2:
            continue
        vert = bm.verts[vert_index]
        touches_mesh_border = any(len(edge.link_faces) == 1 for edge in vert.link_edges)
        if not touches_mesh_border:
            _report_patch_topology_invariant_violation(
                patch_id,
                "L5",
                f"shared_non_border_vertex vert={vert_index} loops={sorted(loop_indices)}",
            )


def _validate_raw_patch_loop_classification(patch_id, patch_data):
    """Validate raw loop classification immediately after the UV boundary step."""

    raw_loops = patch_data.raw_loops
    if not raw_loops:
        _report_patch_topology_invariant_violation(
            patch_id,
            "L1",
            "patch_has_no_raw_loops",
        )
        return

    outer_loop_count = sum(1 for raw_loop in raw_loops if raw_loop.kind == LoopKind.OUTER)
    if outer_loop_count != 1:
        _report_patch_topology_invariant_violation(
            patch_id,
            "L1",
            f"raw_outer_loop_count_mismatch expected=1 actual={outer_loop_count}",
        )

    for loop_index, raw_loop in enumerate(raw_loops):
        if not raw_loop.closed:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "L7",
                "raw_loop_not_closed",
            )

        expected_kind = LoopKind.OUTER if raw_loop.depth % 2 == 0 else LoopKind.HOLE
        if raw_loop.kind != expected_kind:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "L3",
                f"raw_depth_parity_kind_mismatch depth={raw_loop.depth} "
                f"expected={expected_kind.value} actual={raw_loop.kind.value}",
            )

    if len(raw_loops) <= 1:
        return

    planar_results = _snapshot_planar_loop_classification(
        raw_loops,
        patch_data.basis_u,
        patch_data.basis_v,
    )
    uv_results = tuple(
        _LoopClassificationResult(kind=raw_loop.kind, depth=raw_loop.depth)
        for raw_loop in raw_loops
    )
    if planar_results != uv_results:
        uv_kinds = [result.kind.value for result in uv_results]
        uv_depths = [result.depth for result in uv_results]
        planar_kinds = [result.kind.value for result in planar_results]
        planar_depths = [result.depth for result in planar_results]
        _report_loop_classification_diagnostic(
            patch_id,
            f"uv_vs_planar_mismatch uv_kinds={uv_kinds} uv_depths={uv_depths} "
            f"planar_kinds={planar_kinds} planar_depths={planar_depths}",
        )


def _validate_raw_patch_boundary_data(patch_id, patch_data, bm):
    """Run raw patch validation as one explicit topology boundary."""

    _validate_raw_patch_boundary_topology(patch_id, patch_data, bm)
    _validate_raw_patch_loop_classification(patch_id, patch_data)


def _derive_loop_corners_from_final_chains(boundary_loop, basis_u, basis_v, patch_id, loop_index):
    """Derive loop corners strictly from final chain topology, with geometric fallback."""

    chain_count = len(boundary_loop.chains)
    if chain_count < 2:
        return _build_geometric_loop_corners(boundary_loop, basis_u, basis_v)

    corners = []
    for next_chain_index in range(chain_count):
        prev_chain_index = (next_chain_index - 1) % chain_count
        prev_chain = boundary_loop.chains[prev_chain_index]
        next_chain = boundary_loop.chains[next_chain_index]
        corner_identity = _resolve_loop_corner_from_final_chains(
            boundary_loop,
            prev_chain,
            next_chain,
        )
        if not corner_identity.resolved_exactly:
            _report_boundary_loop_invariant_violation(
                patch_id,
                loop_index,
                "R7",
                f"corner_resolution_fallback prev_chain={prev_chain_index} next_chain={next_chain_index} "
                f"prev_end=({prev_chain.end_loop_index},{prev_chain.end_vert_index}) "
                f"next_start=({next_chain.start_loop_index},{next_chain.start_vert_index})",
            )

        loop_vert_index = corner_identity.loop_vert_index
        vert_index = corner_identity.vert_index
        corner_co = corner_identity.vert_co

        prev_point = _find_corner_reference_point(prev_chain.vert_cos, corner_co, reverse=True)
        next_point = _find_corner_reference_point(next_chain.vert_cos, corner_co, reverse=False)
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)

        corners.append(
            BoundaryCorner(
                loop_vert_index=int(loop_vert_index),
                vert_index=int(vert_index),
                vert_co=corner_co,
                prev_chain_index=prev_chain_index,
                next_chain_index=next_chain_index,
                corner_kind=CornerKind.JUNCTION,
                turn_angle_deg=turn_angle_deg,
                prev_role=prev_chain.frame_role,
                next_role=next_chain.frame_role,
            )
        )

    return corners


def _assign_loop_chain_endpoint_topology(boundary_loop):
    """Attach start/end corner indices to every chain in one loop."""

    for chain in boundary_loop.chains:
        chain.start_corner_index = -1
        chain.end_corner_index = -1

    chain_count = len(boundary_loop.chains)
    if chain_count < 2 or len(boundary_loop.corners) != chain_count:
        return

    for corner_index, corner in enumerate(boundary_loop.corners):
        if 0 <= corner.next_chain_index < chain_count:
            boundary_loop.chains[corner.next_chain_index].start_corner_index = corner_index
        if 0 <= corner.prev_chain_index < chain_count:
            boundary_loop.chains[corner.prev_chain_index].end_corner_index = corner_index


def _iter_patch_neighbor_chain_refs(graph):
    """Yield all final chains that claim an opposite patch as neighbor."""

    for patch_id, node in graph.nodes.items():
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                if not chain.has_patch_neighbor:
                    continue
                yield _PatchNeighborChainRef(
                    patch_id=patch_id,
                    loop_index=loop_index,
                    chain_index=chain_index,
                    neighbor_patch_id=chain.neighbor_patch_id,
                    start_vert_index=chain.start_vert_index,
                    end_vert_index=chain.end_vert_index,
                )


def _validate_patch_graph_seam_consistency(graph):
    """Validate that patch-neighbor chains agree with the final seam graph."""

    refs_by_pair: dict[tuple[int, int], list[_PatchNeighborChainRef]] = {}
    for chain_ref in _iter_patch_neighbor_chain_refs(graph):
        pair_key = (
            min(chain_ref.patch_id, chain_ref.neighbor_patch_id),
            max(chain_ref.patch_id, chain_ref.neighbor_patch_id),
        )
        refs_by_pair.setdefault(pair_key, []).append(chain_ref)

    for pair_key, chain_refs in refs_by_pair.items():
        patch_a_id, patch_b_id = pair_key
        seam = graph.get_seam(patch_a_id, patch_b_id)
        if seam is None:
            _report_graph_topology_invariant_violation(
                "X5",
                f"missing_seam pair={pair_key} chain_refs="
                f"{[(ref.patch_id, ref.loop_index, ref.chain_index) for ref in chain_refs]}",
            )
            continue

        shared_vert_indices = set(seam.shared_vert_indices)
        endpoint_pairs_by_patch: dict[int, set[tuple[int, int]]] = {
            patch_a_id: set(),
            patch_b_id: set(),
        }

        for chain_ref in chain_refs:
            if (
                chain_ref.start_vert_index not in shared_vert_indices
                or chain_ref.end_vert_index not in shared_vert_indices
            ):
                _report_graph_topology_invariant_violation(
                    "X6",
                    f"chain_endpoint_outside_seam pair={pair_key} "
                    f"ref=P{chain_ref.patch_id}L{chain_ref.loop_index}C{chain_ref.chain_index} "
                    f"endpoints=({chain_ref.start_vert_index},{chain_ref.end_vert_index}) "
                    f"shared={sorted(shared_vert_indices)}",
                )

            endpoint_pairs_by_patch.setdefault(chain_ref.patch_id, set()).add(chain_ref.endpoint_pair)

        if not endpoint_pairs_by_patch.get(patch_a_id) or not endpoint_pairs_by_patch.get(patch_b_id):
            _report_graph_topology_invariant_violation(
                "X5",
                f"one_sided_patch_neighbor pair={pair_key} "
                f"counts=({len(endpoint_pairs_by_patch.get(patch_a_id, set()))},"
                f"{len(endpoint_pairs_by_patch.get(patch_b_id, set()))})",
            )
            continue

        if endpoint_pairs_by_patch[patch_a_id] != endpoint_pairs_by_patch[patch_b_id]:
            _report_graph_topology_invariant_violation(
                "X6",
                f"endpoint_pair_mismatch pair={pair_key} "
                f"a={sorted(endpoint_pairs_by_patch[patch_a_id])} "
                f"b={sorted(endpoint_pairs_by_patch[patch_b_id])}",
            )


def _begin_patch_topology_assembly(patch_id, patch_face_indices, bm):
    """Create one explicit patch assembly state before graph integration."""

    patch_type, world_facing, normal, area, perimeter = _classify_patch(bm, patch_face_indices)
    basis_u, basis_v = _build_patch_basis(bm, patch_face_indices, patch_type, normal)
    patch_faces = [bm.faces[idx] for idx in patch_face_indices]
    raw_loops = _trace_boundary_loops(patch_faces)
    centroid = _compute_centroid(bm, patch_face_indices)
    mesh_verts, mesh_tris = _serialize_patch_geometry(bm, patch_face_indices)

    node = PatchNode(
        patch_id=patch_id,
        face_indices=list(patch_face_indices),
        centroid=centroid,
        normal=normal,
        area=area,
        perimeter=perimeter,
        patch_type=patch_type,
        world_facing=world_facing,
        basis_u=basis_u,
        basis_v=basis_v,
        boundary_loops=[],
        mesh_verts=mesh_verts,
        mesh_tris=mesh_tris,
    )
    return _PatchTopologyAssemblyState(
        patch_id=patch_id,
        node=node,
        raw_boundary_data=_RawPatchBoundaryData(
            face_indices=list(patch_face_indices),
            raw_loops=raw_loops,
            basis_u=basis_u.copy(),
            basis_v=basis_v.copy(),
        ),
    )


def _build_patch_topology_assembly_states(bm, patches_raw):
    """Build raw patch assembly states before classification / final serialization."""

    return [
        _begin_patch_topology_assembly(patch_id, patch_face_indices, bm)
        for patch_id, patch_face_indices in enumerate(patches_raw)
    ]


def _register_patch_topology_assembly_states(patch_graph, patch_states):
    """Register patch nodes early so face_to_patch exists before boundary serialization."""

    for patch_state in patch_states:
        patch_graph.add_node(patch_state.node)


def _classify_patch_topology_assembly_states(bm, patch_states, obj=None):
    """Run loop-kind classification and raw-boundary validation over assembly states."""

    raw_patch_data = {
        patch_state.patch_id: patch_state.raw_boundary_data
        for patch_state in patch_states
    }
    _classify_loops_outer_hole(bm, raw_patch_data, obj)

    for patch_state in patch_states:
        _validate_raw_patch_boundary_data(
            patch_state.patch_id,
            patch_state.raw_boundary_data,
            bm,
        )


def _finalize_patch_topology_assembly_states(patch_graph, patch_states, bm):
    """Finalize BoundaryLoop serialization for all already-registered patches."""

    for patch_state in patch_states:
        node = patch_state.node
        patch_data = patch_state.raw_boundary_data
        node.boundary_loops = _build_boundary_loops(
            patch_data.raw_loops,
            patch_data.face_indices,
            patch_graph.face_to_patch,
            patch_state.patch_id,
            node.basis_u,
            node.basis_v,
            bm,
        )
        _validate_patch_loop_classification(node)


def _compute_centroid(bm, face_indices):
    """Compute the patch centroid as an average over face vertices."""

    center = Vector((0.0, 0.0, 0.0))
    count = 0
    for face_idx in face_indices:
        face = bm.faces[face_idx]
        for vert in face.verts:
            center += vert.co
            count += 1
    return center / max(count, 1)


def _serialize_patch_geometry(bm, face_indices):
    """Serialize patch geometry into verts/tris for debug and reports."""

    vert_map = {}
    mesh_verts = []
    mesh_tris = []

    for face_idx in face_indices:
        face = bm.faces[face_idx]
        tri = []
        for vert in face.verts:
            if vert.index not in vert_map:
                vert_map[vert.index] = len(mesh_verts)
                mesh_verts.append(vert.co.copy())
            tri.append(vert_map[vert.index])

        if len(tri) == 3:
            mesh_tris.append(tuple(tri))
            continue

        for tri_index in range(1, len(tri) - 1):
            mesh_tris.append((tri[0], tri[tri_index], tri[tri_index + 1]))

    return mesh_verts, mesh_tris


def _build_boundary_loops(raw_loops, patch_face_indices, face_to_patch, patch_id, basis_u, basis_v, bm):
    """Serialize raw loop topology into BoundaryLoop and BoundaryChain objects."""

    boundary_loops = []
    patch_face_indices = set(patch_face_indices)

    for loop_index, raw_loop in enumerate(raw_loops):
        state = _begin_boundary_loop_build(
            raw_loop,
            patch_face_indices,
            face_to_patch,
            patch_id,
            bm,
        )
        # Мержим chains, разделённые бевель-заворотом: поворот вдоль нормали,
        # а не в касательной плоскости. На UV это один непрерывный сегмент.
        _refine_boundary_loop_raw_chains(state, basis_u, basis_v, bm)
        boundary_loops.append(
            _finalize_boundary_loop_build(state, basis_u, basis_v, patch_id, loop_index)
        )

    return boundary_loops


def _classify_loops_outer_hole(bm, raw_patch_data, obj=None):
    """Classify loops as OUTER or HOLE through the explicit UV-dependent analysis boundary."""

    if not raw_patch_data:
        return

    classification_inputs = _prepare_outer_hole_classification_inputs(raw_patch_data)
    if not classification_inputs:
        return

    if obj is None or obj.type != "MESH":
        return

    _classify_multi_loop_patches_via_uv(bm, classification_inputs, obj)


def _build_seam_edges(face_to_patch, bm):
    """Build SeamEdge relations between neighboring patches."""

    seam_links = {}

    for edge in bm.edges:
        if not edge.seam:
            continue

        patch_ids = sorted({
            face_to_patch[face.index]
            for face in edge.link_faces
            if face.index in face_to_patch
        })
        if len(patch_ids) != 2:
            continue

        key = (patch_ids[0], patch_ids[1])
        edge_len = edge.calc_length()
        info = seam_links.setdefault(
            key,
            {
                "shared_length": 0.0,
                "shared_vert_indices": set(),
                "longest_edge_length": -1.0,
                "longest_edge_verts": (0, 0),
            },
        )
        info["shared_length"] += edge_len
        info["shared_vert_indices"].update(vert.index for vert in edge.verts)
        if edge_len > info["longest_edge_length"]:
            info["longest_edge_length"] = edge_len
            info["longest_edge_verts"] = (edge.verts[0].index, edge.verts[1].index)

    return [
        SeamEdge(
            patch_a_id=patch_a_id,
            patch_b_id=patch_b_id,
            shared_length=info["shared_length"],
            shared_vert_indices=sorted(info["shared_vert_indices"]),
            longest_edge_verts=info["longest_edge_verts"],
            longest_edge_length=info["longest_edge_length"],
        )
        for (patch_a_id, patch_b_id), info in seam_links.items()
    ]


def build_patch_graph(bm, face_indices, obj=None):
    """Build a PatchGraph from BMesh patch analysis."""

    bm.faces.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    patch_graph = PatchGraph()
    face_indices = _coerce_face_indices(bm, face_indices)
    if not face_indices:
        return patch_graph

    patches_raw = _flood_fill_patches(bm, face_indices)
    patch_states = _build_patch_topology_assembly_states(bm, patches_raw)
    _register_patch_topology_assembly_states(patch_graph, patch_states)

    # Единственный допустимый side effect внутри analysis: временный UV unwrap
    # для определения OUTER/HOLE у multi-loop boundary.
    _classify_patch_topology_assembly_states(bm, patch_states, obj)
    _finalize_patch_topology_assembly_states(patch_graph, patch_states, bm)

    for seam_edge in _build_seam_edges(patch_graph.face_to_patch, bm):
        patch_graph.add_edge(seam_edge)
    _validate_patch_graph_seam_consistency(patch_graph)

    return patch_graph


def _chain_length(chain):
    """Compute a polyline length for the debug report."""

    if len(chain.vert_cos) < 2:
        return 0.0

    length = 0.0
    for index in range(len(chain.vert_cos) - 1):
        length += (chain.vert_cos[index + 1] - chain.vert_cos[index]).length
    return length


@dataclass(frozen=True)
class _LabelSequenceView:
    labels: tuple[str, ...] = ()


def _build_label_sequence_view(labels):
    return _LabelSequenceView(labels=tuple(labels))


def _format_label_sequence_view(label_view):
    if not label_view.labels:
        return "[]"
    return "[" + " ".join(label_view.labels) + "]"


def _build_chain_ref_labels(chain_refs):
    return _build_label_sequence_view(
        f"L{loop_index}C{chain_index}" for loop_index, chain_index in chain_refs
    )


def _build_frame_run_chain_index_labels(chain_indices):
    return _build_label_sequence_view(
        f"C{chain_index}" for chain_index in chain_indices
    )


@dataclass(frozen=True)
class _JunctionCornerRef:
    patch_id: int
    loop_index: int
    corner_index: int
    prev_chain_index: int
    next_chain_index: int


@dataclass(frozen=True)
class _JunctionChainRef:
    patch_id: int
    loop_index: int
    chain_index: int
    frame_role: FrameRole
    neighbor_kind: ChainNeighborKind


@dataclass(frozen=True)
class _JunctionRunEndpointRef:
    patch_id: int
    loop_index: int
    run_index: int
    dominant_role: FrameRole
    endpoint_kind: str
    corner_index: int


@dataclass(frozen=True)
class _JunctionRolePair:
    prev_role: FrameRole
    next_role: FrameRole


@dataclass
class _JunctionBuildEntry:
    vert_index: int
    vert_co: Vector
    corner_refs: list[_JunctionCornerRef] = field(default_factory=list)
    patch_ids: set[int] = field(default_factory=set)


@dataclass(frozen=True)
class _Junction:
    vert_index: int
    vert_co: Vector
    corner_refs: tuple[_JunctionCornerRef, ...] = ()
    chain_refs: tuple[_JunctionChainRef, ...] = ()
    run_endpoint_refs: tuple[_JunctionRunEndpointRef, ...] = ()
    role_signature: tuple[_JunctionRolePair, ...] = ()
    patch_ids: tuple[int, ...] = ()
    valence: int = 0
    has_mesh_border: bool = False
    has_seam_self: bool = False
    is_open: bool = False
    h_count: int = 0
    v_count: int = 0
    free_count: int = 0


@dataclass(frozen=True)
class _LoopDerivedTopologySummary:
    patch_id: int
    loop_index: int
    kind: LoopKind
    chain_count: int
    corner_count: int
    run_count: int


@dataclass(frozen=True)
class _PatchDerivedTopologySummary:
    patch_id: int
    semantic_key: str
    patch_type: PatchType
    world_facing: WorldFacing
    face_count: int
    loop_kinds: tuple[LoopKind, ...] = ()
    chain_count: int = 0
    corner_count: int = 0
    hole_count: int = 0
    run_count: int = 0
    role_sequence: tuple[FrameRole, ...] = ()
    h_count: int = 0
    v_count: int = 0
    free_count: int = 0
    loop_summaries: tuple[_LoopDerivedTopologySummary, ...] = ()


@dataclass(frozen=True)
class _PatchGraphAggregateCounts:
    total_patches: int = 0
    walls: int = 0
    floors: int = 0
    slopes: int = 0
    singles: int = 0
    total_loops: int = 0
    total_chains: int = 0
    total_corners: int = 0
    total_sharp_corners: int = 0
    total_holes: int = 0
    total_h: int = 0
    total_v: int = 0
    total_free: int = 0
    total_up: int = 0
    total_down: int = 0
    total_side: int = 0
    total_patch_links: int = 0
    total_self_seams: int = 0
    total_mesh_borders: int = 0
    total_run_h: int = 0
    total_run_v: int = 0
    total_run_free: int = 0


@dataclass(frozen=True)
class _PatchGraphDerivedTopology:
    patch_summaries: tuple[_PatchDerivedTopologySummary, ...] = ()
    patch_summaries_by_id: Mapping[int, _PatchDerivedTopologySummary] = field(default_factory=dict)
    loop_summaries_by_key: Mapping[PatchLoopKey, _LoopDerivedTopologySummary] = field(default_factory=dict)
    aggregate_counts: _PatchGraphAggregateCounts = field(default_factory=_PatchGraphAggregateCounts)
    loop_frame_results: Mapping[PatchLoopKey, _LoopFrameRunBuildResult] = field(default_factory=dict)
    frame_runs_by_loop: Mapping[PatchLoopKey, tuple[_FrameRun, ...]] = field(default_factory=dict)
    run_refs_by_corner: Mapping[CornerJunctionKey, tuple["_JunctionRunEndpointRef", ...]] = field(default_factory=dict)
    junctions: tuple[_Junction, ...] = ()
    junctions_by_vert_index: Mapping[int, _Junction] = field(default_factory=dict)


@dataclass(frozen=True)
class _PatchGraphJunctionReportSummary:
    total_junctions: int = 0
    interesting_junctions: tuple[_Junction, ...] = ()
    open_junction_count: int = 0
    valence_histogram: tuple[tuple[int, int], ...] = ()

    @property
    def closed_junction_count(self) -> int:
        return max(0, self.total_junctions - self.open_junction_count)


@dataclass(frozen=True)
class _ChainConsoleView:
    chain_index: int
    role: str
    is_bridge: bool
    neighbor_kind: str
    neighbor_suffix: str
    transition: str
    start_vert_index: int
    end_vert_index: int
    start_corner_label: str
    end_corner_label: str
    edge_count: int
    length_3d: float
    start_endpoint_ref_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    end_endpoint_ref_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)


@dataclass(frozen=True)
class _RunConsoleView:
    run_index: int
    dominant_role: str
    start_corner_label: str
    end_corner_label: str
    total_length: float
    support_length: float
    gap_free_length: float
    max_free_gap_length: float
    projected_u_span: float
    projected_v_span: float
    chain_index_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)


@dataclass(frozen=True)
class _CornerConsoleView:
    corner_index: int
    corner_kind: str
    corner_type: str
    prev_chain_index: int
    next_chain_index: int
    vert_index: int
    turn_angle_deg: float
    is_sharp: bool


@dataclass(frozen=True)
class _LoopConsoleView:
    loop_index: int
    kind: str
    chain_count: int
    corner_count: int
    chain_views: tuple[_ChainConsoleView, ...] = ()
    run_views: tuple[_RunConsoleView, ...] = ()
    corner_views: tuple[_CornerConsoleView, ...] = ()


@dataclass(frozen=True)
class _PatchConsoleView:
    patch_id: int
    semantic_key: str
    patch_type: str
    world_facing: str
    face_count: int
    hole_count: int
    run_count: int
    chain_count: int
    corner_count: int
    h_count: int
    v_count: int
    free_count: int
    loop_kind_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    role_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    loop_views: tuple[_LoopConsoleView, ...] = ()


@dataclass(frozen=True)
class _JunctionConsoleView:
    vert_index: int
    valence: int
    is_open: bool
    has_mesh_border: bool
    has_seam_self: bool
    h_count: int
    v_count: int
    free_count: int
    patch_ref_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    signature_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    corner_ref_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    chain_ref_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    run_ref_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)


@dataclass(frozen=True)
class _PatchGraphSummaryConsoleView:
    valence_labels: _LabelSequenceView = field(default_factory=_LabelSequenceView)
    total_junctions: int = 0
    interesting_junction_count: int = 0
    open_junction_count: int = 0
    closed_junction_count: int = 0


@dataclass(frozen=True)
class _PatchGraphConsoleView:
    patch_views: tuple[_PatchConsoleView, ...] = ()
    junction_views: tuple[_JunctionConsoleView, ...] = ()
    aggregate_counts: _PatchGraphAggregateCounts = field(default_factory=_PatchGraphAggregateCounts)
    junction_summary: _PatchGraphJunctionReportSummary = field(default_factory=_PatchGraphJunctionReportSummary)
    summary_view: _PatchGraphSummaryConsoleView = field(default_factory=_PatchGraphSummaryConsoleView)


def _build_patch_topology_summaries(graph, loop_frame_results):
    """Build canonical per-patch and aggregate topology summaries."""

    patch_summaries: list[_PatchDerivedTopologySummary] = []

    walls = 0
    floors = 0
    slopes = 0
    singles = 0
    total_loops = 0
    total_chains = 0
    total_corners = 0
    total_sharp_corners = 0
    total_holes = 0
    total_h = 0
    total_v = 0
    total_free = 0
    total_up = 0
    total_down = 0
    total_side = 0
    total_patch_links = 0
    total_self_seams = 0
    total_mesh_borders = 0
    total_run_h = 0
    total_run_v = 0
    total_run_free = 0

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        patch_type = node.patch_type
        world_facing = node.world_facing

        if world_facing == WorldFacing.UP:
            total_up += 1
        elif world_facing == WorldFacing.DOWN:
            total_down += 1
        else:
            total_side += 1

        if patch_type == PatchType.WALL:
            walls += 1
        elif patch_type == PatchType.FLOOR:
            floors += 1
        elif patch_type == PatchType.SLOPE:
            slopes += 1

        if len(node.face_indices) == 1:
            singles += 1

        patch_chain_count = 0
        patch_corner_count = 0
        patch_hole_count = 0
        patch_run_count = 0
        patch_h = 0
        patch_v = 0
        patch_free = 0
        role_sequence: list[FrameRole] = []
        loop_kinds: list[LoopKind] = []
        loop_summaries: list[_LoopDerivedTopologySummary] = []

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            loop_kind = boundary_loop.kind
            frame_runs = loop_frame_results.get(
                (patch_id, loop_index),
                _LoopFrameRunBuildResult(effective_roles=(), runs=()),
            ).runs

            total_loops += 1
            patch_chain_count += len(boundary_loop.chains)
            patch_corner_count += len(boundary_loop.corners)
            total_chains += len(boundary_loop.chains)
            total_corners += len(boundary_loop.corners)
            patch_run_count += len(frame_runs)
            loop_kinds.append(loop_kind)

            if loop_kind == LoopKind.HOLE:
                total_holes += 1
                patch_hole_count += 1

            total_sharp_corners += sum(
                1
                for corner in boundary_loop.corners
                if corner.turn_angle_deg >= CORNER_ANGLE_THRESHOLD_DEG
            )

            for chain in boundary_loop.chains:
                role_sequence.append(chain.frame_role)
                if chain.frame_role == FrameRole.H_FRAME:
                    total_h += 1
                    patch_h += 1
                elif chain.frame_role == FrameRole.V_FRAME:
                    total_v += 1
                    patch_v += 1
                else:
                    total_free += 1
                    patch_free += 1

                if chain.neighbor_kind == ChainNeighborKind.PATCH:
                    total_patch_links += 1
                elif chain.neighbor_kind == ChainNeighborKind.SEAM_SELF:
                    total_self_seams += 1
                else:
                    total_mesh_borders += 1

            for frame_run in frame_runs:
                if frame_run.dominant_role == FrameRole.H_FRAME:
                    total_run_h += 1
                elif frame_run.dominant_role == FrameRole.V_FRAME:
                    total_run_v += 1
                else:
                    total_run_free += 1

            loop_summaries.append(
                _LoopDerivedTopologySummary(
                    patch_id=patch_id,
                    loop_index=loop_index,
                    kind=loop_kind,
                    chain_count=len(boundary_loop.chains),
                    corner_count=len(boundary_loop.corners),
                    run_count=len(frame_runs),
                )
            )

        patch_summaries.append(
            _PatchDerivedTopologySummary(
                patch_id=patch_id,
                semantic_key=graph.get_patch_semantic_key(patch_id),
                patch_type=patch_type,
                world_facing=world_facing,
                face_count=len(node.face_indices),
                loop_kinds=tuple(loop_kinds),
                chain_count=patch_chain_count,
                corner_count=patch_corner_count,
                hole_count=patch_hole_count,
                run_count=patch_run_count,
                role_sequence=tuple(role_sequence),
                h_count=patch_h,
                v_count=patch_v,
                free_count=patch_free,
                loop_summaries=tuple(loop_summaries),
            )
        )

    aggregate_counts = _PatchGraphAggregateCounts(
        total_patches=len(graph.nodes),
        walls=walls,
        floors=floors,
        slopes=slopes,
        singles=singles,
        total_loops=total_loops,
        total_chains=total_chains,
        total_corners=total_corners,
        total_sharp_corners=total_sharp_corners,
        total_holes=total_holes,
        total_h=total_h,
        total_v=total_v,
        total_free=total_free,
        total_up=total_up,
        total_down=total_down,
        total_side=total_side,
        total_patch_links=total_patch_links,
        total_self_seams=total_self_seams,
        total_mesh_borders=total_mesh_borders,
        total_run_h=total_run_h,
        total_run_v=total_run_v,
        total_run_free=total_run_free,
    )
    return tuple(patch_summaries), aggregate_counts


def _build_patch_graph_derived_topology(graph):
    """Build the canonical derived topology bundle over the final PatchGraph."""

    loop_frame_results = _build_patch_graph_loop_frame_results(graph)
    patch_summaries, aggregate_counts = _build_patch_topology_summaries(graph, loop_frame_results)
    run_refs_by_corner = _build_junction_run_refs_by_corner(loop_frame_results)
    junctions = tuple(_build_patch_graph_junctions(graph, run_refs_by_corner=run_refs_by_corner))

    patch_summaries_by_id = {
        patch_summary.patch_id: patch_summary
        for patch_summary in patch_summaries
    }
    loop_summaries_by_key = {
        (loop_summary.patch_id, loop_summary.loop_index): loop_summary
        for patch_summary in patch_summaries
        for loop_summary in patch_summary.loop_summaries
    }
    frame_runs_by_loop = {
        loop_key: build_result.runs
        for loop_key, build_result in loop_frame_results.items()
    }
    junctions_by_vert_index = {
        junction.vert_index: junction
        for junction in junctions
    }

    derived_topology = _PatchGraphDerivedTopology(
        patch_summaries=patch_summaries,
        patch_summaries_by_id=MappingProxyType(dict(patch_summaries_by_id)),
        loop_summaries_by_key=MappingProxyType(dict(loop_summaries_by_key)),
        aggregate_counts=aggregate_counts,
        loop_frame_results=MappingProxyType(dict(loop_frame_results)),
        frame_runs_by_loop=MappingProxyType(dict(frame_runs_by_loop)),
        run_refs_by_corner=MappingProxyType(dict(run_refs_by_corner)),
        junctions=junctions,
        junctions_by_vert_index=MappingProxyType(dict(junctions_by_vert_index)),
    )
    _validate_patch_graph_junctions(graph, derived_topology)
    _validate_patch_graph_derived_topology(graph, derived_topology)
    return derived_topology


def _build_junction_run_refs_by_corner(loop_frame_results):
    """Collect frame-run endpoint refs grouped by corner identity."""

    run_refs_by_corner: dict[CornerJunctionKey, list[_JunctionRunEndpointRef]] = {}
    if not loop_frame_results:
        return run_refs_by_corner

    for (patch_id, loop_index), build_result in loop_frame_results.items():
        frame_runs = build_result.runs
        for run_index, frame_run in enumerate(frame_runs):
            endpoint_specs: list[_FrameRunEndpointSpec] = []
            if frame_run.start_corner_index >= 0:
                endpoint_specs.append(
                    _FrameRunEndpointSpec(endpoint_kind="start", corner_index=frame_run.start_corner_index)
                )
            if frame_run.end_corner_index >= 0 and frame_run.end_corner_index != frame_run.start_corner_index:
                endpoint_specs.append(
                    _FrameRunEndpointSpec(endpoint_kind="end", corner_index=frame_run.end_corner_index)
                )
            elif (
                frame_run.end_corner_index >= 0
                and frame_run.end_corner_index == frame_run.start_corner_index
                and endpoint_specs
            ):
                endpoint_specs = [
                    _FrameRunEndpointSpec(endpoint_kind="both", corner_index=frame_run.start_corner_index)
                ]

            for endpoint_spec in endpoint_specs:
                run_refs_by_corner.setdefault(
                    (patch_id, loop_index, endpoint_spec.corner_index),
                    [],
                ).append(
                    _JunctionRunEndpointRef(
                        patch_id=patch_id,
                        loop_index=loop_index,
                        run_index=run_index,
                        dominant_role=frame_run.dominant_role,
                        endpoint_kind=endpoint_spec.endpoint_kind,
                        corner_index=endpoint_spec.corner_index,
                    )
                )
    return {
        corner_key: tuple(run_refs)
        for corner_key, run_refs in run_refs_by_corner.items()
    }


def _junction_run_endpoint_ref_key(run_ref):
    return (run_ref.patch_id, run_ref.loop_index, run_ref.run_index, run_ref.endpoint_kind)


def _junction_corner_ref_key(corner_ref):
    return (corner_ref.patch_id, corner_ref.loop_index, corner_ref.corner_index)


def _junction_chain_ref_key(chain_ref):
    return (chain_ref.patch_id, chain_ref.loop_index, chain_ref.chain_index)


def _derive_junction_role_signature(graph, corner_refs):
    """Derive a deterministic role signature from the concrete corner topology."""

    role_pairs = []
    for corner_ref in corner_refs:
        prev_chain = graph.get_chain(corner_ref.patch_id, corner_ref.loop_index, corner_ref.prev_chain_index)
        next_chain = graph.get_chain(corner_ref.patch_id, corner_ref.loop_index, corner_ref.next_chain_index)
        if prev_chain is None or next_chain is None:
            continue
        role_pairs.append(
            _JunctionRolePair(
                prev_role=prev_chain.frame_role,
                next_role=next_chain.frame_role,
            )
        )
    return tuple(
        sorted(
            role_pairs,
            key=lambda pair: (pair.prev_role.value, pair.next_role.value),
        )
    )


def _derive_junction_chain_refs(graph, corner_refs):
    """Derive unique chain refs touched by the corner set."""

    chain_refs_by_key: dict[tuple[int, int, int], _JunctionChainRef] = {}
    for corner_ref in corner_refs:
        for chain_index in (corner_ref.prev_chain_index, corner_ref.next_chain_index):
            chain = graph.get_chain(corner_ref.patch_id, corner_ref.loop_index, chain_index)
            if chain is None:
                continue
            chain_key = (corner_ref.patch_id, corner_ref.loop_index, chain_index)
            if chain_key in chain_refs_by_key:
                continue
            chain_refs_by_key[chain_key] = _JunctionChainRef(
                patch_id=corner_ref.patch_id,
                loop_index=corner_ref.loop_index,
                chain_index=chain_index,
                frame_role=chain.frame_role,
                neighbor_kind=chain.neighbor_kind,
            )
    return tuple(sorted(
        chain_refs_by_key.values(),
        key=lambda ref: (ref.patch_id, ref.loop_index, ref.chain_index),
    ))


def _derive_junction_run_endpoint_refs(corner_refs, run_refs_by_corner):
    """Derive unique run endpoint refs touched by the corner set."""

    run_endpoint_refs_by_key: dict[tuple[int, int, int, str], _JunctionRunEndpointRef] = {}
    for corner_ref in corner_refs:
        for run_ref in run_refs_by_corner.get(_junction_corner_ref_key(corner_ref), []):
            run_endpoint_refs_by_key[_junction_run_endpoint_ref_key(run_ref)] = run_ref
    return tuple(sorted(
        run_endpoint_refs_by_key.values(),
        key=lambda ref: (ref.patch_id, ref.loop_index, ref.run_index, ref.endpoint_kind),
    ))


def _collect_patch_graph_junction_entries(graph):
    """Collect typed junction build entries from final corner topology."""

    junction_entries: dict[int, _JunctionBuildEntry] = {}
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for corner_index, corner in enumerate(boundary_loop.corners):
                if corner.vert_index < 0:
                    continue

                entry = junction_entries.setdefault(
                    corner.vert_index,
                    _JunctionBuildEntry(
                        vert_index=corner.vert_index,
                        vert_co=corner.vert_co.copy(),
                    ),
                )
                entry.patch_ids.add(patch_id)
                entry.corner_refs.append(
                    _JunctionCornerRef(
                        patch_id=patch_id,
                        loop_index=loop_index,
                        corner_index=corner_index,
                        prev_chain_index=corner.prev_chain_index,
                        next_chain_index=corner.next_chain_index,
                    )
                )
    return junction_entries


def _materialize_junction(graph, entry, run_refs_by_corner):
    """Finalize one junction build entry into the immutable report view."""

    corner_refs = tuple(sorted(
        entry.corner_refs,
        key=lambda ref: (ref.patch_id, ref.loop_index, ref.corner_index),
    ))
    chain_refs = _derive_junction_chain_refs(graph, corner_refs)
    run_endpoint_refs = _derive_junction_run_endpoint_refs(corner_refs, run_refs_by_corner)
    patch_ids = tuple(sorted(entry.patch_ids))
    role_signature = _derive_junction_role_signature(graph, corner_refs)

    h_count = sum(1 for chain_ref in chain_refs if chain_ref.frame_role == FrameRole.H_FRAME)
    v_count = sum(1 for chain_ref in chain_refs if chain_ref.frame_role == FrameRole.V_FRAME)
    free_count = sum(1 for chain_ref in chain_refs if chain_ref.frame_role == FrameRole.FREE)
    has_mesh_border = any(
        chain_ref.neighbor_kind == ChainNeighborKind.MESH_BORDER for chain_ref in chain_refs
    )
    has_seam_self = any(
        chain_ref.neighbor_kind == ChainNeighborKind.SEAM_SELF for chain_ref in chain_refs
    )

    return _Junction(
        vert_index=entry.vert_index,
        vert_co=entry.vert_co.copy(),
        corner_refs=corner_refs,
        chain_refs=chain_refs,
        run_endpoint_refs=run_endpoint_refs,
        role_signature=role_signature,
        patch_ids=patch_ids,
        valence=len(corner_refs),
        has_mesh_border=has_mesh_border,
        has_seam_self=has_seam_self,
        is_open=has_mesh_border,
        h_count=h_count,
        v_count=v_count,
        free_count=free_count,
    )


def _build_patch_graph_junctions(graph, run_refs_by_corner=None):
    """Build a diagnostic-only junction view from final patch/loop/corner topology."""

    run_refs_by_corner = run_refs_by_corner or {}
    junction_entries = _collect_patch_graph_junction_entries(graph)
    return [
        _materialize_junction(graph, junction_entries[vert_index], run_refs_by_corner)
        for vert_index in sorted(junction_entries.keys())
    ]


def _validate_patch_graph_junctions(graph, derived_topology):
    """Validate the derived junction layer against final corner/chain/run topology."""

    junctions = derived_topology.junctions
    corner_to_junction: dict[CornerJunctionKey, int] = {}
    loop_frame_results = derived_topology.loop_frame_results or {}
    run_refs_by_corner = derived_topology.run_refs_by_corner or {}
    for junction in junctions:
        if junction.valence != len(junction.corner_refs):
            _report_junction_invariant_violation(
                junction.vert_index,
                "J2",
                f"valence_mismatch expected={len(junction.corner_refs)} actual={junction.valence}",
            )

        expected_patch_ids = sorted({corner_ref.patch_id for corner_ref in junction.corner_refs})
        if junction.patch_ids != expected_patch_ids:
            _report_junction_invariant_violation(
                junction.vert_index,
                "J4",
                f"patch_ids_mismatch expected={expected_patch_ids} actual={junction.patch_ids}",
            )

        if junction.is_open != junction.has_mesh_border:
            _report_junction_invariant_violation(
                junction.vert_index,
                "J3",
                f"open_flag_mismatch is_open={junction.is_open} has_mesh_border={junction.has_mesh_border}",
            )

        expected_chain_keys: set[tuple[int, int, int]] = set()
        expected_run_ref_keys: set[tuple[int, int, int, str]] = set()
        for corner_ref in junction.corner_refs:
            corner_key = (corner_ref.patch_id, corner_ref.loop_index, corner_ref.corner_index)
            if corner_key in corner_to_junction:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J1",
                    f"corner_shared_between_junctions corner={corner_key} other=V{corner_to_junction[corner_key]}",
                )
                continue
            corner_to_junction[corner_key] = junction.vert_index

            node = graph.nodes.get(corner_ref.patch_id)
            if node is None or corner_ref.loop_index >= len(node.boundary_loops):
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J4",
                    f"missing_corner_patch corner={corner_key}",
                )
                continue
            boundary_loop = node.boundary_loops[corner_ref.loop_index]
            if corner_ref.corner_index >= len(boundary_loop.corners):
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J4",
                    f"missing_corner_index corner={corner_key}",
                )
                continue
            corner = boundary_loop.corners[corner_ref.corner_index]
            if corner.vert_index != junction.vert_index:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J4",
                    f"corner_vert_mismatch corner={corner_key} actual_vert={corner.vert_index}",
                )
            if (corner.vert_co - junction.vert_co).length > 1e-5:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J4",
                    f"corner_co_mismatch corner={corner_key}",
                )

            expected_chain_keys.add((corner_ref.patch_id, corner_ref.loop_index, corner_ref.prev_chain_index))
            expected_chain_keys.add((corner_ref.patch_id, corner_ref.loop_index, corner_ref.next_chain_index))
            for run_ref in run_refs_by_corner.get(corner_key, []):
                expected_run_ref_keys.add(_junction_run_endpoint_ref_key(run_ref))

        actual_chain_keys = {_junction_chain_ref_key(chain_ref) for chain_ref in junction.chain_refs}
        if actual_chain_keys != expected_chain_keys:
            _report_junction_invariant_violation(
                junction.vert_index,
                "J3",
                f"chain_ref_set_mismatch expected={sorted(expected_chain_keys)} actual={sorted(actual_chain_keys)}",
            )

        actual_run_ref_keys = {
            _junction_run_endpoint_ref_key(run_ref) for run_ref in junction.run_endpoint_refs
        }
        if actual_run_ref_keys != expected_run_ref_keys:
            _report_junction_invariant_violation(
                junction.vert_index,
                "J3",
                f"run_ref_set_mismatch expected={sorted(expected_run_ref_keys)} actual={sorted(actual_run_ref_keys)}",
            )

        expected_role_signature = _derive_junction_role_signature(graph, junction.corner_refs)
        if junction.role_signature != expected_role_signature:
            _report_junction_invariant_violation(
                junction.vert_index,
                "J5",
                f"role_signature_mismatch expected={_format_label_sequence_view(_build_junction_role_signature_labels(expected_role_signature))} "
                f"actual={_format_label_sequence_view(_build_junction_role_signature_labels(junction.role_signature))}",
            )

        for chain_ref in junction.chain_refs:
            chain = graph.get_chain(chain_ref.patch_id, chain_ref.loop_index, chain_ref.chain_index)
            if chain is None:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"missing_chain_ref ref=P{chain_ref.patch_id}L{chain_ref.loop_index}C{chain_ref.chain_index}",
                )
                continue
            if junction.vert_index not in (chain.start_vert_index, chain.end_vert_index):
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"chain_not_touching_junction ref=P{chain_ref.patch_id}L{chain_ref.loop_index}C{chain_ref.chain_index} "
                    f"endpoints=({chain.start_vert_index},{chain.end_vert_index})",
                )

        for run_ref in junction.run_endpoint_refs:
            frame_runs = loop_frame_results.get(
                (run_ref.patch_id, run_ref.loop_index),
                _LoopFrameRunBuildResult(effective_roles=(), runs=()),
            ).runs
            if run_ref.run_index < 0 or run_ref.run_index >= len(frame_runs):
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"missing_run_ref ref=P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index}",
                )
                continue
            frame_run = frame_runs[run_ref.run_index]
            if frame_run.dominant_role != run_ref.dominant_role:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"run_role_mismatch ref=P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index} "
                    f"expected={frame_run.dominant_role.value} actual={run_ref.dominant_role.value}",
                )
            if run_ref.endpoint_kind == "start" and frame_run.start_corner_index != run_ref.corner_index:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"run_start_corner_mismatch ref=P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index} "
                    f"expected={frame_run.start_corner_index} actual={run_ref.corner_index}",
                )
            if run_ref.endpoint_kind == "end" and frame_run.end_corner_index != run_ref.corner_index:
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"run_end_corner_mismatch ref=P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index} "
                    f"expected={frame_run.end_corner_index} actual={run_ref.corner_index}",
                )
            if run_ref.endpoint_kind == "both" and (
                frame_run.start_corner_index != run_ref.corner_index
                or frame_run.end_corner_index != run_ref.corner_index
            ):
                _report_junction_invariant_violation(
                    junction.vert_index,
                    "J3",
                    f"run_both_corner_mismatch ref=P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index} "
                    f"start={frame_run.start_corner_index} end={frame_run.end_corner_index} actual={run_ref.corner_index}",
                )

    for patch_id, node in graph.nodes.items():
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for corner_index, corner in enumerate(boundary_loop.corners):
                if corner.vert_index < 0:
                    continue
                corner_key = (patch_id, loop_index, corner_index)
                if corner_key not in corner_to_junction:
                    _report_junction_invariant_violation(
                        corner.vert_index,
                        "J4",
                        f"corner_missing_from_junctions corner={corner_key}",
                    )
    return junctions


def _validate_patch_graph_derived_topology(graph, derived_topology):
    """Validate the canonical derived topology bundle against the final PatchGraph."""

    patch_summaries = derived_topology.patch_summaries
    patch_summaries_by_id = derived_topology.patch_summaries_by_id
    loop_summaries_by_key = derived_topology.loop_summaries_by_key
    loop_frame_results = derived_topology.loop_frame_results
    frame_runs_by_loop = derived_topology.frame_runs_by_loop
    junctions = derived_topology.junctions
    junctions_by_vert_index = derived_topology.junctions_by_vert_index
    run_refs_by_corner = derived_topology.run_refs_by_corner
    aggregate = derived_topology.aggregate_counts

    expected_patch_ids = tuple(sorted(summary.patch_id for summary in patch_summaries))
    actual_patch_ids = tuple(sorted(patch_summaries_by_id.keys()))
    if actual_patch_ids != expected_patch_ids:
        _report_graph_topology_invariant_violation(
            "D1",
            f"patch_summary_index_mismatch expected={expected_patch_ids} actual={actual_patch_ids}",
        )

    expected_loop_keys: set[PatchLoopKey] = set()
    expected_run_keys: set[PatchLoopKey] = set()
    total_chains = 0
    total_corners = 0
    total_holes = 0
    total_sharp_corners = 0
    total_h = 0
    total_v = 0
    total_free = 0
    total_patch_links = 0
    total_self_seams = 0
    total_mesh_borders = 0
    total_runs = 0
    total_run_h = 0
    total_run_v = 0
    total_run_free = 0

    patch_summary_order = tuple(summary.patch_id for summary in patch_summaries)
    if patch_summary_order != tuple(sorted(patch_summary_order)):
        _report_graph_topology_invariant_violation(
            "D1",
            f"patch_summary_order_mismatch actual={patch_summary_order}",
        )

    for patch_summary in patch_summaries:
        indexed_patch_summary = patch_summaries_by_id.get(patch_summary.patch_id)
        if indexed_patch_summary != patch_summary:
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_summary_lookup_mismatch patch={patch_summary.patch_id}",
            )

        node = graph.nodes.get(patch_summary.patch_id)
        if node is None:
            _report_graph_topology_invariant_violation(
                "D1",
                f"missing_patch_node patch={patch_summary.patch_id}",
            )
            continue

        if patch_summary.face_count != len(node.face_indices):
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_face_count_mismatch patch={patch_summary.patch_id} "
                f"expected={len(node.face_indices)} actual={patch_summary.face_count}",
            )

        if len(patch_summary.loop_summaries) != len(node.boundary_loops):
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_loop_count_mismatch patch={patch_summary.patch_id} "
                f"expected={len(node.boundary_loops)} actual={len(patch_summary.loop_summaries)}",
            )

        patch_chain_count = 0
        patch_corner_count = 0
        patch_hole_count = 0
        patch_run_count = 0
        patch_h = 0
        patch_v = 0
        patch_free = 0
        loop_summary_order = tuple(loop_summary.loop_index for loop_summary in patch_summary.loop_summaries)
        if loop_summary_order != tuple(sorted(loop_summary_order)):
            _report_graph_topology_invariant_violation(
                "D2",
                f"loop_summary_order_mismatch patch={patch_summary.patch_id} actual={loop_summary_order}",
            )

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            loop_key = (patch_summary.patch_id, loop_index)
            expected_loop_keys.add(loop_key)
            patch_chain_count += len(boundary_loop.chains)
            patch_corner_count += len(boundary_loop.corners)
            total_chains += len(boundary_loop.chains)
            total_corners += len(boundary_loop.corners)
            total_sharp_corners += sum(
                1
                for corner in boundary_loop.corners
                if corner.turn_angle_deg >= CORNER_ANGLE_THRESHOLD_DEG
            )

            indexed_loop_summary = loop_summaries_by_key.get(loop_key)
            if indexed_loop_summary is None:
                _report_graph_topology_invariant_violation(
                    "D2",
                    f"missing_loop_summary loop=P{patch_summary.patch_id}L{loop_index}",
                )
                continue

            expected_loop_summary = patch_summary.loop_summaries[loop_index] if loop_index < len(patch_summary.loop_summaries) else None
            if expected_loop_summary != indexed_loop_summary:
                _report_graph_topology_invariant_violation(
                    "D2",
                    f"loop_summary_lookup_mismatch loop=P{patch_summary.patch_id}L{loop_index}",
                )

            if indexed_loop_summary.kind != boundary_loop.kind:
                _report_graph_topology_invariant_violation(
                    "D2",
                    f"loop_kind_mismatch loop=P{patch_summary.patch_id}L{loop_index} "
                    f"expected={boundary_loop.kind.value} actual={indexed_loop_summary.kind.value}",
                )
            if boundary_loop.kind == LoopKind.HOLE:
                patch_hole_count += 1
                total_holes += 1
            if indexed_loop_summary.chain_count != len(boundary_loop.chains):
                _report_graph_topology_invariant_violation(
                    "D2",
                    f"loop_chain_count_mismatch loop=P{patch_summary.patch_id}L{loop_index} "
                    f"expected={len(boundary_loop.chains)} actual={indexed_loop_summary.chain_count}",
                )
            if indexed_loop_summary.corner_count != len(boundary_loop.corners):
                _report_graph_topology_invariant_violation(
                    "D2",
                    f"loop_corner_count_mismatch loop=P{patch_summary.patch_id}L{loop_index} "
                    f"expected={len(boundary_loop.corners)} actual={indexed_loop_summary.corner_count}",
                )

            frame_result = loop_frame_results.get(loop_key)
            if frame_result is None:
                _report_graph_topology_invariant_violation(
                    "D3",
                    f"missing_loop_frame_result loop=P{patch_summary.patch_id}L{loop_index}",
                )
                continue
            expected_run_keys.add(loop_key)

            indexed_frame_runs = frame_runs_by_loop.get(loop_key)
            if indexed_frame_runs != frame_result.runs:
                _report_graph_topology_invariant_violation(
                    "D3",
                    f"frame_run_lookup_mismatch loop=P{patch_summary.patch_id}L{loop_index}",
                )

            if indexed_loop_summary.run_count != len(frame_result.runs):
                _report_graph_topology_invariant_violation(
                    "D2",
                    f"loop_run_count_mismatch loop=P{patch_summary.patch_id}L{loop_index} "
                    f"expected={len(frame_result.runs)} actual={indexed_loop_summary.run_count}",
                )
            patch_run_count += len(frame_result.runs)

            for chain in boundary_loop.chains:
                if chain.frame_role == FrameRole.H_FRAME:
                    patch_h += 1
                    total_h += 1
                elif chain.frame_role == FrameRole.V_FRAME:
                    patch_v += 1
                    total_v += 1
                else:
                    patch_free += 1
                    total_free += 1

                if chain.neighbor_kind == ChainNeighborKind.PATCH:
                    total_patch_links += 1
                elif chain.neighbor_kind == ChainNeighborKind.SEAM_SELF:
                    total_self_seams += 1
                else:
                    total_mesh_borders += 1

            for frame_run in frame_result.runs:
                total_runs += 1
                if frame_run.dominant_role == FrameRole.H_FRAME:
                    total_run_h += 1
                elif frame_run.dominant_role == FrameRole.V_FRAME:
                    total_run_v += 1
                else:
                    total_run_free += 1

        if patch_summary.chain_count != patch_chain_count:
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_chain_count_mismatch patch={patch_summary.patch_id} "
                f"expected={patch_chain_count} actual={patch_summary.chain_count}",
            )
        if patch_summary.corner_count != patch_corner_count:
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_corner_count_mismatch patch={patch_summary.patch_id} "
                f"expected={patch_corner_count} actual={patch_summary.corner_count}",
            )
        if patch_summary.hole_count != patch_hole_count:
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_hole_count_mismatch patch={patch_summary.patch_id} "
                f"expected={patch_hole_count} actual={patch_summary.hole_count}",
            )
        if patch_summary.run_count != patch_run_count:
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_run_count_mismatch patch={patch_summary.patch_id} "
                f"expected={patch_run_count} actual={patch_summary.run_count}",
            )
        if patch_summary.h_count != patch_h or patch_summary.v_count != patch_v or patch_summary.free_count != patch_free:
            _report_graph_topology_invariant_violation(
                "D1",
                f"patch_role_count_mismatch patch={patch_summary.patch_id} "
                f"expected=({patch_h},{patch_v},{patch_free}) actual=({patch_summary.h_count},{patch_summary.v_count},{patch_summary.free_count})",
            )

    actual_loop_keys = set(loop_summaries_by_key.keys())
    if actual_loop_keys != expected_loop_keys:
        _report_graph_topology_invariant_violation(
            "D2",
            f"loop_summary_keyset_mismatch expected={sorted(expected_loop_keys)} actual={sorted(actual_loop_keys)}",
        )

    actual_run_keys = set(frame_runs_by_loop.keys())
    if actual_run_keys != expected_run_keys or set(loop_frame_results.keys()) != expected_run_keys:
        _report_graph_topology_invariant_violation(
            "D3",
            f"frame_run_keyset_mismatch expected={sorted(expected_run_keys)} actual={sorted(actual_run_keys)} "
            f"build={sorted(loop_frame_results.keys())}",
        )

    expected_junction_vert_indices = tuple(sorted(junction.vert_index for junction in junctions))
    if tuple(junction.vert_index for junction in junctions) != expected_junction_vert_indices:
        _report_graph_topology_invariant_violation(
            "D4",
            f"junction_order_mismatch actual={tuple(junction.vert_index for junction in junctions)}",
        )
    actual_junction_vert_indices = tuple(sorted(junctions_by_vert_index.keys()))
    if actual_junction_vert_indices != expected_junction_vert_indices:
        _report_graph_topology_invariant_violation(
            "D4",
            f"junction_index_mismatch expected={expected_junction_vert_indices} actual={actual_junction_vert_indices}",
        )
    for junction in junctions:
        indexed_junction = junctions_by_vert_index.get(junction.vert_index)
        if indexed_junction != junction:
            _report_graph_topology_invariant_violation(
                "D4",
                f"junction_lookup_mismatch vert={junction.vert_index}",
            )

    for corner_key in run_refs_by_corner.keys():
        patch_id, loop_index, corner_index = corner_key
        node = graph.nodes.get(patch_id)
        if node is None or loop_index >= len(node.boundary_loops):
            _report_graph_topology_invariant_violation(
                "D5",
                f"run_ref_missing_loop corner=P{patch_id}L{loop_index}K{corner_index}",
            )
            continue
        boundary_loop = node.boundary_loops[loop_index]
        if corner_index < 0 or corner_index >= len(boundary_loop.corners):
            _report_graph_topology_invariant_violation(
                "D5",
                f"run_ref_missing_corner corner=P{patch_id}L{loop_index}K{corner_index}",
            )

    if aggregate.total_patches != len(patch_summaries):
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_patch_count_mismatch expected={len(patch_summaries)} actual={aggregate.total_patches}",
        )
    if aggregate.total_loops != len(loop_summaries_by_key):
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_loop_count_mismatch expected={len(loop_summaries_by_key)} actual={aggregate.total_loops}",
        )
    if aggregate.total_chains != total_chains or aggregate.total_corners != total_corners or aggregate.total_holes != total_holes:
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_topology_count_mismatch expected=({total_chains},{total_corners},{total_holes}) "
            f"actual=({aggregate.total_chains},{aggregate.total_corners},{aggregate.total_holes})",
        )
    if aggregate.total_sharp_corners != total_sharp_corners:
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_sharp_corner_count_mismatch expected={total_sharp_corners} actual={aggregate.total_sharp_corners}",
        )
    if aggregate.total_h != total_h or aggregate.total_v != total_v or aggregate.total_free != total_free:
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_role_count_mismatch expected=({total_h},{total_v},{total_free}) "
            f"actual=({aggregate.total_h},{aggregate.total_v},{aggregate.total_free})",
        )
    if (
        aggregate.total_patch_links != total_patch_links
        or aggregate.total_self_seams != total_self_seams
        or aggregate.total_mesh_borders != total_mesh_borders
    ):
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_neighbor_count_mismatch expected=({total_patch_links},{total_self_seams},{total_mesh_borders}) "
            f"actual=({aggregate.total_patch_links},{aggregate.total_self_seams},{aggregate.total_mesh_borders})",
        )
    if aggregate.total_run_h != total_run_h or aggregate.total_run_v != total_run_v or aggregate.total_run_free != total_run_free:
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_run_role_count_mismatch expected=({total_run_h},{total_run_v},{total_run_free}) "
            f"actual=({aggregate.total_run_h},{aggregate.total_run_v},{aggregate.total_run_free})",
        )
    if aggregate.total_run_h + aggregate.total_run_v + aggregate.total_run_free != total_runs:
        _report_graph_topology_invariant_violation(
            "D6",
            f"aggregate_run_total_mismatch expected={total_runs} "
            f"actual={aggregate.total_run_h + aggregate.total_run_v + aggregate.total_run_free}",
        )


def _build_patch_id_labels(patch_ids):
    return _build_label_sequence_view(
        f"P{patch_id}" for patch_id in patch_ids
    )


def _build_junction_corner_ref_labels(corner_refs):
    return _build_label_sequence_view(
        f"P{corner_ref.patch_id}L{corner_ref.loop_index}K{corner_ref.corner_index}"
        for corner_ref in corner_refs
    )


def _build_junction_chain_ref_labels(chain_refs):
    return _build_label_sequence_view(
        f"P{chain_ref.patch_id}L{chain_ref.loop_index}C{chain_ref.chain_index}"
        for chain_ref in chain_refs
    )


def _build_junction_run_ref_labels(run_endpoint_refs):
    return _build_label_sequence_view(
        f"P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index}:{run_ref.endpoint_kind}"
        for run_ref in run_endpoint_refs
    )


def _build_junction_role_signature_labels(role_signature):
    return _build_label_sequence_view(
        f"{role_pair.prev_role.value}->{role_pair.next_role.value}"
        for role_pair in role_signature
    )


def _build_patch_graph_junction_report_summary(derived_topology):
    """Build presentation-only junction aggregates over canonical derived topology."""

    interesting_junctions = sorted(
        [
            junction
            for junction in derived_topology.junctions
            if junction.valence >= 2 or junction.has_seam_self
        ],
        key=lambda junction: (
            0 if not junction.is_open else 1,
            -junction.valence,
            junction.vert_index,
        ),
    )

    valence_histogram: dict[int, int] = {}
    open_junction_count = 0
    for junction in derived_topology.junctions:
        valence_histogram[junction.valence] = valence_histogram.get(junction.valence, 0) + 1
        if junction.is_open:
            open_junction_count += 1

    return _PatchGraphJunctionReportSummary(
        total_junctions=len(derived_topology.junctions),
        interesting_junctions=tuple(interesting_junctions),
        open_junction_count=open_junction_count,
        valence_histogram=tuple(sorted(valence_histogram.items())),
    )


def _build_chain_console_view(graph, patch_id, loop_index, chain_index, chain):
    """Build one typed chain console view."""

    role = _enum_value(chain.frame_role)
    neighbor_kind = _enum_value(chain.neighbor_kind)
    neighbor_suffix = ""
    if neighbor_kind == ChainNeighborKind.PATCH.value:
        neighbor_node = graph.nodes.get(chain.neighbor_patch_id)
        neighbor_semantic = graph.get_patch_semantic_key(chain.neighbor_patch_id) if neighbor_node else "UNKNOWN"
        neighbor_suffix = f" -> Patch {chain.neighbor_patch_id}:{neighbor_semantic}"

    endpoint_neighbors = graph.get_chain_endpoint_neighbors(patch_id, loop_index, chain_index)
    return _ChainConsoleView(
        chain_index=chain_index,
        role=role,
        is_bridge=(role == FrameRole.FREE.value and len(chain.vert_cos) <= 2),
        neighbor_kind=neighbor_kind,
        neighbor_suffix=neighbor_suffix,
        transition=graph.describe_chain_transition(patch_id, chain),
        start_vert_index=chain.start_vert_index,
        end_vert_index=chain.end_vert_index,
        start_corner_label=str(chain.start_corner_index) if chain.start_corner_index >= 0 else "-",
        end_corner_label=str(chain.end_corner_index) if chain.end_corner_index >= 0 else "-",
        edge_count=len(chain.edge_indices),
        length_3d=_chain_length(chain),
        start_endpoint_ref_labels=_build_chain_ref_labels(endpoint_neighbors['start']),
        end_endpoint_ref_labels=_build_chain_ref_labels(endpoint_neighbors['end']),
    )


def _build_run_console_view(run_index, frame_run):
    """Build one typed frame-run console view."""

    return _RunConsoleView(
        run_index=run_index,
        dominant_role=frame_run.dominant_role.value,
        start_corner_label=str(frame_run.start_corner_index) if frame_run.start_corner_index >= 0 else "-",
        end_corner_label=str(frame_run.end_corner_index) if frame_run.end_corner_index >= 0 else "-",
        total_length=frame_run.total_length,
        support_length=frame_run.support_length,
        gap_free_length=frame_run.gap_free_length,
        max_free_gap_length=frame_run.max_free_gap_length,
        projected_u_span=frame_run.projected_u_span,
        projected_v_span=frame_run.projected_v_span,
        chain_index_labels=_build_frame_run_chain_index_labels(frame_run.chain_indices),
    )


def _build_corner_console_view(corner_index, corner):
    """Build one typed corner console view."""

    return _CornerConsoleView(
        corner_index=corner_index,
        corner_kind=corner.corner_kind.value,
        corner_type=corner.corner_type,
        prev_chain_index=corner.prev_chain_index,
        next_chain_index=corner.next_chain_index,
        vert_index=corner.vert_index,
        turn_angle_deg=corner.turn_angle_deg,
        is_sharp=(corner.turn_angle_deg >= CORNER_ANGLE_THRESHOLD_DEG),
    )


def _build_loop_console_view(graph, patch_id, loop_summary, boundary_loop, frame_runs_by_loop):
    """Build one typed loop console view."""

    chain_views = tuple(
        _build_chain_console_view(graph, patch_id, loop_summary.loop_index, chain_index, chain)
        for chain_index, chain in enumerate(boundary_loop.chains)
    )
    run_views = tuple(
        _build_run_console_view(run_index, frame_run)
        for run_index, frame_run in enumerate(frame_runs_by_loop.get((patch_id, loop_summary.loop_index), ()))
    )
    corner_views = tuple(
        _build_corner_console_view(corner_index, corner)
        for corner_index, corner in enumerate(boundary_loop.corners)
    )
    return _LoopConsoleView(
        loop_index=loop_summary.loop_index,
        kind=_enum_value(loop_summary.kind),
        chain_count=loop_summary.chain_count,
        corner_count=loop_summary.corner_count,
        chain_views=chain_views,
        run_views=run_views,
        corner_views=corner_views,
    )


def _build_patch_console_view(graph, patch_summary, frame_runs_by_loop):
    """Build one typed patch console view in canonical derived order."""

    node = graph.nodes.get(patch_summary.patch_id)
    if node is None:
        return None

    loop_views = []
    for loop_summary in patch_summary.loop_summaries:
        loop_index = loop_summary.loop_index
        if loop_index < 0 or loop_index >= len(node.boundary_loops):
            continue
        loop_views.append(
            _build_loop_console_view(
                graph,
                patch_summary.patch_id,
                loop_summary,
                node.boundary_loops[loop_index],
                frame_runs_by_loop,
            )
        )

    return _PatchConsoleView(
        patch_id=patch_summary.patch_id,
        semantic_key=patch_summary.semantic_key,
        patch_type=_enum_value(patch_summary.patch_type),
        world_facing=_enum_value(patch_summary.world_facing),
        face_count=patch_summary.face_count,
        hole_count=patch_summary.hole_count,
        run_count=patch_summary.run_count,
        chain_count=patch_summary.chain_count,
        corner_count=patch_summary.corner_count,
        h_count=patch_summary.h_count,
        v_count=patch_summary.v_count,
        free_count=patch_summary.free_count,
        loop_kind_labels=_build_label_sequence_view(
            _enum_value(kind) for kind in patch_summary.loop_kinds
        ),
        role_labels=_build_label_sequence_view(
            _enum_value(role) for role in patch_summary.role_sequence
        ),
        loop_views=tuple(loop_views),
    )


def _build_junction_console_view(junction):
    """Build one typed junction console view."""

    return _JunctionConsoleView(
        vert_index=junction.vert_index,
        valence=junction.valence,
        is_open=junction.is_open,
        has_mesh_border=junction.has_mesh_border,
        has_seam_self=junction.has_seam_self,
        h_count=junction.h_count,
        v_count=junction.v_count,
        free_count=junction.free_count,
        patch_ref_labels=_build_patch_id_labels(junction.patch_ids),
        signature_labels=_build_junction_role_signature_labels(junction.role_signature),
        corner_ref_labels=_build_junction_corner_ref_labels(junction.corner_refs),
        chain_ref_labels=_build_junction_chain_ref_labels(junction.chain_refs),
        run_ref_labels=_build_junction_run_ref_labels(junction.run_endpoint_refs),
    )


def _build_patch_graph_summary_console_view(junction_summary):
    """Build typed summary/count labels over canonical junction report summary."""

    return _PatchGraphSummaryConsoleView(
        valence_labels=_build_label_sequence_view(
            f"v{valence}:{count}" for valence, count in junction_summary.valence_histogram
        ),
        total_junctions=junction_summary.total_junctions,
        interesting_junction_count=len(junction_summary.interesting_junctions),
        open_junction_count=junction_summary.open_junction_count,
        closed_junction_count=junction_summary.closed_junction_count,
    )


def _validate_patch_graph_console_view(derived_topology, console_view):
    """Validate the typed console/snapshot view against canonical derived topology."""

    patch_views = console_view.patch_views
    patch_summaries = derived_topology.patch_summaries
    if tuple(patch_view.patch_id for patch_view in patch_views) != tuple(
        patch_summary.patch_id for patch_summary in patch_summaries
    ):
        _report_graph_topology_invariant_violation(
            "RV1",
            f"patch_view_order_mismatch actual={tuple(patch_view.patch_id for patch_view in patch_views)} "
            f"expected={tuple(patch_summary.patch_id for patch_summary in patch_summaries)}",
        )

    for patch_view, patch_summary in zip(patch_views, patch_summaries):
        if patch_view.semantic_key != patch_summary.semantic_key:
            _report_graph_topology_invariant_violation(
                "RV1",
                f"patch_semantic_key_mismatch patch={patch_summary.patch_id} "
                f"expected={patch_summary.semantic_key} actual={patch_view.semantic_key}",
            )
        if (
            patch_view.face_count != patch_summary.face_count
            or patch_view.hole_count != patch_summary.hole_count
            or patch_view.run_count != patch_summary.run_count
            or patch_view.chain_count != patch_summary.chain_count
            or patch_view.corner_count != patch_summary.corner_count
        ):
            _report_graph_topology_invariant_violation(
                "RV1",
                f"patch_count_mismatch patch={patch_summary.patch_id}",
            )
        if (
            patch_view.h_count != patch_summary.h_count
            or patch_view.v_count != patch_summary.v_count
            or patch_view.free_count != patch_summary.free_count
        ):
            _report_graph_topology_invariant_violation(
                "RV1",
                f"patch_role_count_mismatch patch={patch_summary.patch_id}",
            )
        expected_loop_kind_labels = _build_label_sequence_view(
            _enum_value(kind) for kind in patch_summary.loop_kinds
        )
        if patch_view.loop_kind_labels != expected_loop_kind_labels:
            _report_graph_topology_invariant_violation(
                "RV1",
                f"patch_loop_kind_labels_mismatch patch={patch_summary.patch_id}",
            )
        expected_role_labels = _build_label_sequence_view(
            _enum_value(role) for role in patch_summary.role_sequence
        )
        if patch_view.role_labels != expected_role_labels:
            _report_graph_topology_invariant_violation(
                "RV1",
                f"patch_role_labels_mismatch patch={patch_summary.patch_id}",
            )
        if len(patch_view.loop_views) != len(patch_summary.loop_summaries):
            _report_graph_topology_invariant_violation(
                "RV2",
                f"patch_loop_view_count_mismatch patch={patch_summary.patch_id} "
                f"expected={len(patch_summary.loop_summaries)} actual={len(patch_view.loop_views)}",
            )

        if tuple(loop_view.loop_index for loop_view in patch_view.loop_views) != tuple(
            loop_summary.loop_index for loop_summary in patch_summary.loop_summaries
        ):
            _report_graph_topology_invariant_violation(
                "RV2",
                f"loop_view_order_mismatch patch={patch_summary.patch_id} "
                f"actual={tuple(loop_view.loop_index for loop_view in patch_view.loop_views)} "
                f"expected={tuple(loop_summary.loop_index for loop_summary in patch_summary.loop_summaries)}",
            )

        for loop_view, loop_summary in zip(patch_view.loop_views, patch_summary.loop_summaries):
            if (
                loop_view.kind != _enum_value(loop_summary.kind)
                or loop_view.chain_count != loop_summary.chain_count
                or loop_view.corner_count != loop_summary.corner_count
            ):
                _report_graph_topology_invariant_violation(
                    "RV2",
                    f"loop_view_mismatch patch={patch_summary.patch_id} loop={loop_summary.loop_index}",
                )
            if len(loop_view.run_views) != loop_summary.run_count:
                _report_graph_topology_invariant_violation(
                    "RV2",
                    f"loop_run_view_count_mismatch patch={patch_summary.patch_id} loop={loop_summary.loop_index} "
                    f"expected={loop_summary.run_count} actual={len(loop_view.run_views)}",
                )

    junction_summary = console_view.junction_summary
    if tuple(junction_view.vert_index for junction_view in console_view.junction_views) != tuple(
        junction.vert_index for junction in junction_summary.interesting_junctions
    ):
        _report_graph_topology_invariant_violation(
            "RV3",
            f"junction_view_order_mismatch actual={tuple(junction_view.vert_index for junction_view in console_view.junction_views)} "
            f"expected={tuple(junction.vert_index for junction in junction_summary.interesting_junctions)}",
        )

    summary_view = console_view.summary_view
    expected_valence_labels = _build_label_sequence_view(
        f"v{valence}:{count}" for valence, count in junction_summary.valence_histogram
    )
    if summary_view.valence_labels != expected_valence_labels:
        _report_graph_topology_invariant_violation(
            "RV3",
            "valence_labels_mismatch",
        )
    if (
        summary_view.total_junctions != junction_summary.total_junctions
        or summary_view.interesting_junction_count != len(junction_summary.interesting_junctions)
        or summary_view.open_junction_count != junction_summary.open_junction_count
        or summary_view.closed_junction_count != junction_summary.closed_junction_count
    ):
        _report_graph_topology_invariant_violation(
            "RV3",
            "junction_summary_view_mismatch",
        )


def _build_patch_graph_console_view(graph, derived_topology):
    """Build typed console/snapshot view rows over canonical derived topology."""

    junction_summary = _build_patch_graph_junction_report_summary(derived_topology)
    patch_views = tuple(
        patch_view
        for patch_view in (
            _build_patch_console_view(graph, patch_summary, derived_topology.frame_runs_by_loop)
            for patch_summary in derived_topology.patch_summaries
        )
        if patch_view is not None
    )
    junction_views = tuple(
        _build_junction_console_view(junction)
        for junction in junction_summary.interesting_junctions
    )
    console_view = _PatchGraphConsoleView(
        patch_views=patch_views,
        junction_views=junction_views,
        aggregate_counts=derived_topology.aggregate_counts,
        junction_summary=junction_summary,
        summary_view=_build_patch_graph_summary_console_view(junction_summary),
    )
    _validate_patch_graph_console_view(derived_topology, console_view)
    return console_view


def _serialize_chain_console_view(chain_view):
    """Serialize one typed chain console view into a stable report line."""

    bridge_tag = " [BRIDGE]" if chain_view.is_bridge else ""
    return (
        f"      Chain {chain_view.chain_index}: {chain_view.role}{bridge_tag} | "
        f"neighbor:{chain_view.neighbor_kind}{chain_view.neighbor_suffix} | transition:{chain_view.transition} | "
        f"verts:{chain_view.start_vert_index}->{chain_view.end_vert_index} | "
        f"corners:{chain_view.start_corner_label}->{chain_view.end_corner_label} | "
        f"ep:start{_format_label_sequence_view(chain_view.start_endpoint_ref_labels)} "
        f"end{_format_label_sequence_view(chain_view.end_endpoint_ref_labels)} | "
        f"edges:{chain_view.edge_count} | length:{chain_view.length_3d:.4f}"
    )


def _serialize_run_console_view(run_view):
    """Serialize one typed frame-run console view into a stable report line."""

    return (
        f"      Run {run_view.run_index}: {run_view.dominant_role} | "
        f"chains:{_format_label_sequence_view(run_view.chain_index_labels)} | "
        f"corners:{run_view.start_corner_label}->{run_view.end_corner_label} | "
        f"total:{run_view.total_length:.4f} support:{run_view.support_length:.4f} "
        f"free_gap:{run_view.gap_free_length:.4f} max_gap:{run_view.max_free_gap_length:.4f} | "
        f"span:u={run_view.projected_u_span:.4f} v={run_view.projected_v_span:.4f}"
    )


def _serialize_corner_console_view(corner_view):
    """Serialize one typed corner console view into a stable report line."""

    return (
        f"      Corner {corner_view.corner_index}: {corner_view.corner_kind} {corner_view.corner_type} | "
        f"chains:{corner_view.prev_chain_index}->{corner_view.next_chain_index} | "
        f"vert:{corner_view.vert_index} | turn:{corner_view.turn_angle_deg:.1f} | "
        f"sharp:{'Y' if corner_view.is_sharp else 'N'}"
    )


def _serialize_loop_console_view(loop_view):
    """Serialize one typed loop console view into stable report lines."""

    lines = [
        f"    Loop {loop_view.loop_index}: {loop_view.kind} | chains:{loop_view.chain_count} corners:{loop_view.corner_count}"
    ]
    lines.extend(_serialize_chain_console_view(chain_view) for chain_view in loop_view.chain_views)
    lines.extend(_serialize_run_console_view(run_view) for run_view in loop_view.run_views)
    lines.extend(_serialize_corner_console_view(corner_view) for corner_view in loop_view.corner_views)
    return lines


def _serialize_patch_console_view(patch_view):
    """Serialize one typed patch console view into stable report lines."""

    lines = [
        f"  Patch {patch_view.patch_id}: {patch_view.patch_type} | facing:{patch_view.world_facing} | {patch_view.face_count}f | "
        f"loops:{len(patch_view.loop_views)}{_format_label_sequence_view(patch_view.loop_kind_labels)} "
        f"chains:{patch_view.chain_count} corners:{patch_view.corner_count} | "
        f"roles:{_format_label_sequence_view(patch_view.role_labels)}"
    ]
    for loop_view in patch_view.loop_views:
        lines.extend(_serialize_loop_console_view(loop_view))
    return lines


def _serialize_junction_console_view(junction_view):
    """Serialize one typed junction console view into a stable report line."""

    return (
        f"    Junction V{junction_view.vert_index}: valence:{junction_view.valence} "
        f"patches:{_format_label_sequence_view(junction_view.patch_ref_labels)} "
        f"open:{'Y' if junction_view.is_open else 'N'} border:{'Y' if junction_view.has_mesh_border else 'N'} "
        f"self:{'Y' if junction_view.has_seam_self else 'N'} | "
        f"roles:H{junction_view.h_count} V{junction_view.v_count} F{junction_view.free_count} | "
        f"signature:{_format_label_sequence_view(junction_view.signature_labels)} | "
        f"corners:{_format_label_sequence_view(junction_view.corner_ref_labels)} | "
        f"chains:{_format_label_sequence_view(junction_view.chain_ref_labels)} | "
        f"runs:{_format_label_sequence_view(junction_view.run_ref_labels)}"
    )


def _serialize_patch_graph_report_summary(console_view):
    """Serialize the full report summary over typed console view counts."""

    aggregate = console_view.aggregate_counts
    summary_view = console_view.summary_view
    return (
        f"Patches: {aggregate.total_patches} (W:{aggregate.walls} F:{aggregate.floors} S:{aggregate.slopes} 1f:{aggregate.singles}) | "
        f"Loops: {aggregate.total_loops} Chains: {aggregate.total_chains} Corners: {aggregate.total_corners} "
        f"Sharp:{aggregate.total_sharp_corners} Holes: {aggregate.total_holes} | "
        f"Roles: H:{aggregate.total_h} V:{aggregate.total_v} Free:{aggregate.total_free} | "
        f"Facing: Up:{aggregate.total_up} Down:{aggregate.total_down} Side:{aggregate.total_side} | "
        f"Neighbors: Patch:{aggregate.total_patch_links} Self:{aggregate.total_self_seams} Border:{aggregate.total_mesh_borders} | "
        f"Junctions: {summary_view.total_junctions} Interesting:{summary_view.interesting_junction_count}"
    )


def _serialize_patch_graph_snapshot_summary(console_view):
    """Serialize the stable snapshot summary over typed console view counts."""

    aggregate = console_view.aggregate_counts
    summary_view = console_view.summary_view
    return (
        f"PatchGraph snapshot | patches:{aggregate.total_patches} loops:{aggregate.total_loops} chains:{aggregate.total_chains} "
        f"corners:{aggregate.total_corners} runs:{aggregate.total_run_h + aggregate.total_run_v + aggregate.total_run_free} "
        f"junctions:{summary_view.total_junctions}"
    )


def _serialize_patch_graph_topology_line(console_view):
    """Serialize the snapshot topology totals line."""

    aggregate = console_view.aggregate_counts
    return (
        f"patches:{aggregate.total_patches} loops:{aggregate.total_loops} chains:{aggregate.total_chains} "
        f"corners:{aggregate.total_corners} holes:{aggregate.total_holes}"
    )


def _serialize_patch_graph_roles_line(console_view):
    """Serialize the snapshot role totals line."""

    aggregate = console_view.aggregate_counts
    return (
        f"H:{aggregate.total_h} V:{aggregate.total_v} Free:{aggregate.total_free} | "
        f"Runs: H:{aggregate.total_run_h} V:{aggregate.total_run_v} Free:{aggregate.total_run_free}"
    )


def _serialize_patch_graph_neighbors_line(console_view):
    """Serialize the snapshot neighbor totals line."""

    aggregate = console_view.aggregate_counts
    return f"Patch:{aggregate.total_patch_links} Self:{aggregate.total_self_seams} Border:{aggregate.total_mesh_borders}"


def _serialize_patch_graph_junctions_line(console_view):
    """Serialize the snapshot junction summary line."""

    summary_view = console_view.summary_view
    return (
        f"total:{summary_view.total_junctions} interesting:{summary_view.interesting_junction_count} "
        f"open:{summary_view.open_junction_count} closed:{summary_view.closed_junction_count} "
        f"valence:{_format_label_sequence_view(summary_view.valence_labels)}"
    )


def _serialize_patch_graph_report_lines(console_view, mesh_name=None):
    """Serialize the full typed console view into System Console report lines."""

    lines = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    for patch_view in console_view.patch_views:
        lines.extend(_serialize_patch_console_view(patch_view))

    if console_view.junction_views:
        lines.append(
            f"  Junctions: {_serialize_patch_graph_junctions_line(console_view)}"
        )
        lines.extend(
            _serialize_junction_console_view(junction_view)
            for junction_view in console_view.junction_views
        )

    return lines


def _serialize_patch_graph_snapshot_lines(console_view, mesh_name=None):
    """Serialize the typed console view into stable snapshot lines."""

    lines = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    for patch_view in console_view.patch_views:
        lines.append(
            f"P{patch_view.patch_id} {patch_view.semantic_key} | "
            f"faces:{patch_view.face_count} loops:{len(patch_view.loop_views)} "
            f"holes:{patch_view.hole_count} "
            f"chains:{patch_view.chain_count} corners:{patch_view.corner_count} "
            f"runs:{patch_view.run_count} | "
            f"roles:H{patch_view.h_count} V{patch_view.v_count} F{patch_view.free_count}"
        )

    lines.append(f"Topology: {_serialize_patch_graph_topology_line(console_view)}")
    lines.append(f"Roles: {_serialize_patch_graph_roles_line(console_view)}")
    lines.append(f"Neighbors: {_serialize_patch_graph_neighbors_line(console_view)}")
    lines.append(f"Junctions: {_serialize_patch_graph_junctions_line(console_view)}")
    for junction_view in console_view.junction_views:
        lines.append(
            f"JV{junction_view.vert_index} valence:{junction_view.valence} "
            f"open:{'Y' if junction_view.is_open else 'N'} border:{'Y' if junction_view.has_mesh_border else 'N'} "
            f"self:{'Y' if junction_view.has_seam_self else 'N'} "
            f"patches:{_format_label_sequence_view(junction_view.patch_ref_labels)} "
            f"signature:{_format_label_sequence_view(junction_view.signature_labels)} "
            f"corners:{_format_label_sequence_view(junction_view.corner_ref_labels)}"
        )

    return lines


def format_patch_graph_report(graph, mesh_name=None) -> FormattedReport:
    """Build text lines for the System Console PatchGraph report."""

    derived_topology = _build_patch_graph_derived_topology(graph)
    console_view = _build_patch_graph_console_view(graph, derived_topology)
    return FormattedReport(
        lines=_serialize_patch_graph_report_lines(console_view, mesh_name=mesh_name),
        summary=_serialize_patch_graph_report_summary(console_view),
    )


def format_patch_graph_snapshot_report(graph, mesh_name=None) -> FormattedReport:
    """Build a compact, stable PatchGraph snapshot for regression baselines."""

    derived_topology = _build_patch_graph_derived_topology(graph)
    console_view = _build_patch_graph_console_view(graph, derived_topology)
    return FormattedReport(
        lines=_serialize_patch_graph_snapshot_lines(console_view, mesh_name=mesh_name),
        summary=_serialize_patch_graph_snapshot_summary(console_view),
    )





