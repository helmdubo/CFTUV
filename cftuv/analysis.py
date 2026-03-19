from __future__ import annotations

from dataclasses import dataclass, field
import math

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


@dataclass
class _AnalysisUvClassificationState:
    """Rollback contract for the only UV-dependent step inside analysis."""

    temp_uv_name: str = ""
    original_active_uv_name: str | None = None
    original_selection: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class _CornerTurnCandidate:
    index: int
    turn_angle_deg: float


@dataclass(frozen=True)
class _BoundarySideKey:
    face_index: int
    edge_index: int
    vert_index: int


@dataclass(frozen=True)
class _UvPoint2D:
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


DirectionBucketKey = tuple[float, float, float]
PatchLoopKey = tuple[int, int]
CornerJunctionKey = tuple[int, int, int]


@dataclass(frozen=True)
class _FrameRunGroup:
    dominant_role: FrameRole
    chain_indices: list[int]


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


def _split_loop_into_chains_by_neighbor(loop_vert_indices, loop_vert_cos, loop_edge_indices, loop_side_face_indices, loop_neighbors):
    """Split a boundary loop into chains when the neighbor changes."""

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


def _merge_bevel_wrap_chains(chains, bm):
    """Мержит соседние chains, разделённые бевель-заворотом поверхности.

    После split по neighbor, бевель-edge создаёт split point где neighbor
    меняется (MESH_BORDER → SEAM_SELF). Но если поворот в этой точке
    происходит вдоль нормали (бевель-заворот), а не в касательной плоскости,
    chains нужно объединить — на UV это один непрерывный сегмент.
    """
    if len(chains) < 2:
        return chains

    bm.verts.ensure_lookup_table()

    def _should_merge(chain_a, chain_b):
        """Проверяет, нужно ли мержить chain_a и chain_b в точке стыка.

        Используем vertex normal (BMVert.normal) в точке стыка — это Blender-овская
        усреднённая нормаль по всем смежным faces, стабильная даже на стыке
        граней с разной ориентацией (бевель-угол).
        """
        cos_a = chain_a.vert_cos
        cos_b = chain_b.vert_cos
        if len(cos_a) < 2 or len(cos_b) < 2:
            return False

        # Точка стыка = последняя вершина chain_a = первая вершина chain_b
        corner_co = cos_a[-1]
        prev_point = cos_a[-2]
        next_point = cos_b[1] if len(cos_b) > 1 else cos_b[0]

        # Vertex normal в точке стыка
        vert_index = chain_a.vert_indices[-1]
        if vert_index >= len(bm.verts):
            return False
        vert_normal = bm.verts[vert_index].normal
        if vert_normal.length_squared < 1e-12:
            return False

        return not _is_tangent_plane_turn(corner_co, prev_point, next_point, vert_normal)

    def _do_merge(chain_a, chain_b):
        """Объединяет chain_b в chain_a (chain_b's first vertex = chain_a's last)."""
        return _RawBoundaryChain(
            vert_indices=chain_a.vert_indices + chain_b.vert_indices[1:],
            vert_cos=chain_a.vert_cos + chain_b.vert_cos[1:],
            edge_indices=chain_a.edge_indices + chain_b.edge_indices,
            side_face_indices=chain_a.side_face_indices + chain_b.side_face_indices[1:],
            neighbor=chain_a.neighbor,
            is_closed=chain_a.is_closed,
            start_loop_index=chain_a.start_loop_index,
            end_loop_index=chain_b.end_loop_index,
        )

    # Итеративно мержим пока есть что мержить
    merged = True
    result = list(chains)
    while merged:
        merged = False
        new_result = []
        i = 0
        while i < len(result):
            if i + 1 < len(result) and _should_merge(result[i], result[i + 1]):
                vi = result[i].vert_indices[-1]
                print(f"  [BevelMerge] merging chains at vert {vi}: "
                      f"{len(result[i].vert_indices)}v + {len(result[i + 1].vert_indices)}v")
                new_result.append(_do_merge(result[i], result[i + 1]))
                i += 2
                merged = True
            else:
                new_result.append(result[i])
                i += 1
        # Проверяем wrap-around: последний chain → первый chain
        if len(new_result) >= 2 and _should_merge(new_result[-1], new_result[0]):
            merged_chain = _do_merge(new_result[-1], new_result[0])
            new_result = [merged_chain] + new_result[1:-1]
            merged = True
        result = new_result

    return result


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
                return _UvPoint2D(uv.x, uv.y)

        vert = bm.verts[vert_index]
        uvs = [loop[uv_layer].uv for loop in vert.link_loops if loop.face.index in patch_face_indices]
        if not uvs:
            return _UvPoint2D(0.0, 0.0)
        return _UvPoint2D(
            sum(uv.x for uv in uvs) / len(uvs),
            sum(uv.y for uv in uvs) / len(uvs),
        )

    polys_2d = []
    for loop_index, raw_loop in enumerate(raw_loops):
        poly = []
        for face_index, edge_index, vert_index in zip(
            raw_loop.side_face_indices,
            raw_loop.edge_indices,
            raw_loop.vert_indices,
        ):
            poly.append(get_side_uv(face_index, edge_index, vert_index))
        polys_2d.append(poly)

    def signed_area(poly):
        area = 0.0
        count = len(poly)
        for idx in range(count):
            x1, y1 = poly[idx].x, poly[idx].y
            x2, y2 = poly[(idx + 1) % count].x, poly[(idx + 1) % count].y
            area += x1 * y2 - x2 * y1
        return 0.5 * area

    def point_in_poly(point, poly):
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

    def interior_point(poly):
        count = len(poly)
        if count < 3:
            return poly[0] if poly else _UvPoint2D(0.0, 0.0)

        signed = signed_area(poly)
        edges_by_len: list[_PolygonEdgeLengthCandidate] = []
        for idx in range(count):
            next_idx = (idx + 1) % count
            dx = poly[next_idx].x - poly[idx].x
            dy = poly[next_idx].y - poly[idx].y
            edges_by_len.append(_PolygonEdgeLengthCandidate(dx * dx + dy * dy, idx))
        edges_by_len.sort(reverse=True)

        for edge_candidate in edges_by_len:
            next_idx = (edge_candidate.index + 1) % count
            mid_x = (poly[edge_candidate.index].x + poly[next_idx].x) * 0.5
            mid_y = (poly[edge_candidate.index].y + poly[next_idx].y) * 0.5
            dx = poly[next_idx].x - poly[edge_candidate.index].x
            dy = poly[next_idx].y - poly[edge_candidate.index].y
            edge_len = math.sqrt(edge_candidate.len_squared)
            if edge_len < 1e-12:
                continue

            if signed >= 0.0:
                normal_x, normal_y = -dy / edge_len, dx / edge_len
            else:
                normal_x, normal_y = dy / edge_len, -dx / edge_len

            epsilon = edge_len * 0.01
            point = _UvPoint2D(mid_x + normal_x * epsilon, mid_y + normal_y * epsilon)
            if point_in_poly(point, poly):
                return point

        return _UvPoint2D(
            sum(point.x for point in poly) / count,
            sum(point.y for point in poly) / count,
        )

    interior_points = [interior_point(poly) for poly in polys_2d]

    for loop_index, raw_loop in enumerate(raw_loops):
        depth = 0
        for poly_index, poly in enumerate(polys_2d):
            if loop_index == poly_index:
                continue
            if point_in_poly(interior_points[loop_index], poly):
                depth += 1

        raw_loop.depth = depth
        raw_loop.kind = LoopKind.OUTER if depth == 0 or depth % 2 == 0 else LoopKind.HOLE


def _prepare_outer_hole_classification_inputs(raw_patch_data):
    """Normalize trivial loop kinds and collect only multi-loop patches for UV classification."""

    classification_inputs = []

    for patch_data in raw_patch_data.values():
        if len(patch_data.raw_loops) <= 1:
            for raw_loop in patch_data.raw_loops:
                raw_loop.kind = LoopKind.OUTER
                raw_loop.depth = 0
            continue
        classification_inputs.append(patch_data)

    return classification_inputs


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
    """Enter the only allowed UV-mutation boundary inside analysis."""

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
    """Find stable corner indices for a closed OUTER loop.

    Это intentionally stricter path, чем `_find_open_chain_corners()`.
    Здесь split меняет loop-wide topology, поэтому нужен стабильный набор
    углов по всему closed loop, с wrap-aware spacing и фильтром бевель-заворотов.
    """

    vertex_count = len(loop_vert_cos)
    if vertex_count < 4:
        return []

    # Подготовка vertex normals для фильтрации бевель-заворотов
    # Vertex normal (BMVert.normal) — усреднённая по всем смежным faces,
    # стабильна на стыке разно-ориентированных граней (бевель-угол).
    use_vert_normals = (loop_vert_indices is not None and bm is not None)
    if use_vert_normals:
        bm.verts.ensure_lookup_table()

    raw_candidate_corners: list[_CornerTurnCandidate] = []
    candidate_corners: list[_CornerTurnCandidate] = []
    for loop_vert_index in range(vertex_count):
        corner_co = loop_vert_cos[loop_vert_index]
        prev_point = loop_vert_cos[(loop_vert_index - 1) % vertex_count]
        next_point = loop_vert_cos[(loop_vert_index + 1) % vertex_count]
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)
        if turn_angle_deg < CORNER_ANGLE_THRESHOLD_DEG:
            continue
        raw_candidate_corners.append(_CornerTurnCandidate(loop_vert_index, turn_angle_deg))
        # Фильтр: пропускаем бевель-завороты (поворот вдоль нормали)
        if use_vert_normals:
            vi = loop_vert_indices[loop_vert_index]
            if vi < len(bm.verts):
                vn = bm.verts[vi].normal
                if vn.length_squared > 1e-12 and not _is_tangent_plane_turn(corner_co, prev_point, next_point, vn):
                    continue
        candidate_corners.append(_CornerTurnCandidate(loop_vert_index, turn_angle_deg))

    if len(candidate_corners) < 4 and len(raw_candidate_corners) >= 4:
        non_hairpin_candidates = [
            candidate
            for candidate in raw_candidate_corners
            if candidate.turn_angle_deg < 150.0
        ]
        # Для curved wall strips real rectangle corners могут быть валидными в 2D,
        # но vertex-normal filter режет их как bevel-like. В таком случае делаем
        # fallback только на non-hairpin corners и дальше всё ещё прогоняем через
        # обычный cluster/spacing filter.
        if len(non_hairpin_candidates) >= 4:
            print(f"[CFTUV][GeoSplit] fallback raw_non_hairpin candidates={len(non_hairpin_candidates)}")
            candidate_corners = non_hairpin_candidates

    if len(candidate_corners) < 4:
        return []

    def _filter_corner_clusters(min_span_length: float, min_vertex_gap: int) -> list[_CornerTurnCandidate]:
        filtered: list[_CornerTurnCandidate] = []
        for candidate in candidate_corners:
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

    perimeter = _loop_arc_length(loop_vert_cos, 0, 0)
    min_vertex_gap = 1
    strict_min_span_length = max(perimeter * 0.04, 1e-4)
    filtered_corners = _filter_corner_clusters(strict_min_span_length, min_vertex_gap)

    # У вытянутых тонких loops короткая сторона может быть меньше 4% perimeter.
    # В этом случае strict spacing схлопывает две реальные corner одной короткой
    # стороны в один cluster, и rectangle-like wall strip не разваливается в 4 chains.
    # Если сильных candidate corners уже >= 4, но strict pass их перемержил, делаем
    # один консервативный retry с меньшим span threshold.
    if len(filtered_corners) < 4 and len(candidate_corners) >= 4:
        relaxed_min_span_length = max(perimeter * 0.015, 1e-4)
        if relaxed_min_span_length < strict_min_span_length:
            relaxed_corners = _filter_corner_clusters(relaxed_min_span_length, min_vertex_gap)
            if len(relaxed_corners) >= 4:
                filtered_corners = relaxed_corners

    if len(filtered_corners) < 4:
        return []

    return [candidate.index for candidate in filtered_corners]


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
    """Detect corners on an open (non-closed) chain by per-vertex angle.

    Returns list of indices into vert_cos where turn angle >= CORNER_ANGLE_THRESHOLD_DEG.
    Excludes endpoints (0 and len-1) — they are chain boundaries, not interior corners.
    Бевель-завороты (поворот вдоль vertex normal) отфильтровываются если переданы
    vert_indices и bm.

    Это intentionally more local / permissive path, чем
    `_collect_geometric_split_indices()`: open border chains уже существуют как
    отдельные сегменты и мы only ищем interior corner split points внутри них.
    """
    vertex_count = len(vert_cos)
    if vertex_count < 3:
        return []

    use_vert_normals = (vert_indices is not None and bm is not None)

    candidates: list[_CornerTurnCandidate] = []
    for i in range(1, vertex_count - 1):
        turn_angle = _measure_corner_turn_angle(vert_cos[i], vert_cos[i - 1], vert_cos[i + 1], basis_u, basis_v)
        print(f"[CFTUV][CornerDetect] idx={i} turn={turn_angle:.1f} threshold={CORNER_ANGLE_THRESHOLD_DEG} "
              f"use_normals={use_vert_normals}")
        if turn_angle >= CORNER_ANGLE_THRESHOLD_DEG:
            # Фильтр: бевель-завороты
            if use_vert_normals:
                vi = vert_indices[i]
                if vi < len(bm.verts):
                    vn = bm.verts[vi].normal
                    if vn.length_squared > 1e-12 and not _is_tangent_plane_turn(
                            vert_cos[i], vert_cos[i - 1], vert_cos[i + 1], vn):
                        continue
        candidates.append(_CornerTurnCandidate(i, turn_angle))

    if not candidates:
        return []

    # Фильтрация: min_spacing вершин между соседними углами
    filtered = [candidates[0]]
    for i in range(1, len(candidates)):
        candidate = candidates[i]
        prev_candidate = filtered[-1]
        if candidate.index - prev_candidate.index < min_spacing:
            if candidate.turn_angle_deg > prev_candidate.turn_angle_deg:
                filtered[-1] = candidate
            continue
        filtered.append(candidate)

    return [candidate.index for candidate in filtered]


def _find_open_chain_corners_filtered(vert_cos, basis_u, basis_v, min_spacing=2,
                                      vert_indices=None, bm=None):
    """Detect open-border corners with a guard against projected smooth hairpins."""

    vertex_count = len(vert_cos)
    if vertex_count < 3:
        return []

    use_vert_normals = (vert_indices is not None and bm is not None)

    candidates: list[_CornerTurnCandidate] = []
    for i in range(1, vertex_count - 1):
        turn_angle = _measure_corner_turn_angle(vert_cos[i], vert_cos[i - 1], vert_cos[i + 1], basis_u, basis_v)
        turn_angle_3d = _measure_corner_turn_angle_3d(vert_cos[i], vert_cos[i - 1], vert_cos[i + 1])
        print(f"[CFTUV][CornerDetect] idx={i} turn={turn_angle:.1f} threshold={CORNER_ANGLE_THRESHOLD_DEG} "
              f"use_normals={use_vert_normals}")
        if turn_angle < CORNER_ANGLE_THRESHOLD_DEG:
            continue

        is_hairpin_turn = turn_angle >= 150.0
        if is_hairpin_turn and turn_angle_3d < 120.0:
            print(f"[CFTUV][CornerDetect] skip idx={i} reason=projected_hairpin "
                  f"turn2d={turn_angle:.1f} turn3d={turn_angle_3d:.1f}")
            continue

        if use_vert_normals and is_hairpin_turn:
            vi = vert_indices[i]
            if vi < len(bm.verts):
                vn = bm.verts[vi].normal
                if vn.length_squared > 1e-12 and not _is_tangent_plane_turn(
                        vert_cos[i], vert_cos[i - 1], vert_cos[i + 1], vn):
                    print(f"[CFTUV][CornerDetect] skip idx={i} reason=hairpin_not_tangent "
                          f"turn2d={turn_angle:.1f} turn3d={turn_angle_3d:.1f}")
                    continue

            candidates.append(_CornerTurnCandidate(i, turn_angle))

    if not candidates:
        return []

    filtered = [candidates[0]]
    for i in range(1, len(candidates)):
        candidate = candidates[i]
        prev_candidate = filtered[-1]
        if candidate.index - prev_candidate.index < min_spacing:
            if candidate.turn_angle_deg > prev_candidate.turn_angle_deg:
                filtered[-1] = candidate
            continue
        filtered.append(candidate)

    return [candidate.index for candidate in filtered]


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
        corner_indices = _find_open_chain_corners_filtered(
            vert_cos, basis_u, basis_v,
            min_spacing=1,
            vert_indices=raw_chain.vert_indices,
            bm=bm,
            # Не передаём bm/vert_indices — бевель-фильтр отвергает
            # реальные UV-углы на краях меша, где vertex normal
            # усреднена с бевель-гранями.
        )

        # Debug: показать что corner detection нашёл
        corner_indices = _filter_open_chain_corner_indices_with_support(
            raw_chain, corner_indices, basis_u, basis_v
        )

        points_2d = [Vector((co.dot(basis_u), co.dot(basis_v))) for co in vert_cos]
        print(f"[CFTUV][BorderSplit] MESH_BORDER chain verts={len(vert_cos)} corners={corner_indices} "
              f"pts={[(round(p.x,3), round(p.y,3)) for p in points_2d]}")

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
    split_indices = _collect_geometric_split_indices(
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
    chain_indices: list[int] = field(default_factory=list)
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
        chain_indices=list(chain_indices),
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


def _build_frame_runs(boundary_loop, basis_u, basis_v, patch_id, loop_index):
    """Build diagnostic continuity runs over final chains of one loop."""

    chains = list(boundary_loop.chains)
    if not chains:
        return []

    effective_roles = []
    for chain_index, chain in enumerate(chains):
        inferred_gap_role = _infer_frame_run_gap_role(chains, chain_index, basis_u, basis_v)
        effective_roles.append(inferred_gap_role if inferred_gap_role is not None else chain.frame_role)

    run_chain_indices: list[_FrameRunGroup] = []
    current_indices = [0]
    current_role = effective_roles[0]

    for chain_index in range(1, len(chains)):
        if effective_roles[chain_index] == current_role:
            current_indices.append(chain_index)
            continue
        run_chain_indices.append(_FrameRunGroup(dominant_role=current_role, chain_indices=current_indices))
        current_indices = [chain_index]
        current_role = effective_roles[chain_index]

    run_chain_indices.append(_FrameRunGroup(dominant_role=current_role, chain_indices=current_indices))

    if (
        len(run_chain_indices) > 1
        and run_chain_indices[0].dominant_role == run_chain_indices[-1].dominant_role
    ):
        merged_role = run_chain_indices[-1].dominant_role
        merged_indices = run_chain_indices[-1].chain_indices + run_chain_indices[0].chain_indices
        run_chain_indices = [
            _FrameRunGroup(dominant_role=merged_role, chain_indices=merged_indices)
        ] + run_chain_indices[1:-1]

    frame_runs = []
    for run_group in run_chain_indices:
        frame_run = _build_frame_run(
            chains,
            effective_roles,
            basis_u,
            basis_v,
            patch_id,
            loop_index,
            run_group.chain_indices,
        )
        if frame_run is not None:
            frame_runs.append(frame_run)
    return frame_runs


def _build_patch_graph_frame_runs(graph):
    """Build diagnostic FrameRun views for every loop in the PatchGraph."""

    frame_runs_by_loop: dict[PatchLoopKey, list[_FrameRun]] = {}
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            frame_runs_by_loop[(patch_id, loop_index)] = _build_frame_runs(
                boundary_loop,
                node.basis_u,
                node.basis_v,
                patch_id,
                loop_index,
            )
    return frame_runs_by_loop


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
    """Resolve corner identity only from the final chain representation."""

    vertex_count = len(boundary_loop.vert_indices)
    shared_vert_index = -1
    if prev_chain.end_vert_index >= 0 and prev_chain.end_vert_index == next_chain.start_vert_index:
        shared_vert_index = prev_chain.end_vert_index
    elif next_chain.start_vert_index >= 0:
        shared_vert_index = next_chain.start_vert_index
    elif prev_chain.end_vert_index >= 0:
        shared_vert_index = prev_chain.end_vert_index

    candidate_loop_indices = []
    if vertex_count > 0:
        candidate_loop_indices.append(next_chain.start_loop_index % vertex_count)
        prev_end_loop_index = prev_chain.end_loop_index % vertex_count
        if prev_end_loop_index not in candidate_loop_indices:
            candidate_loop_indices.append(prev_end_loop_index)

    loop_vert_index = -1
    for candidate_loop_index in candidate_loop_indices:
        if candidate_loop_index < 0 or candidate_loop_index >= vertex_count:
            continue
        if shared_vert_index >= 0 and boundary_loop.vert_indices[candidate_loop_index] != shared_vert_index:
            continue
        loop_vert_index = candidate_loop_index
        break

    if loop_vert_index < 0 and shared_vert_index >= 0:
        for candidate_loop_index, vert_index in enumerate(boundary_loop.vert_indices):
            if vert_index == shared_vert_index:
                loop_vert_index = candidate_loop_index
                break

    if 0 <= loop_vert_index < len(boundary_loop.vert_cos):
        corner_co = boundary_loop.vert_cos[loop_vert_index].copy()
        vert_index = boundary_loop.vert_indices[loop_vert_index]
        return loop_vert_index, vert_index, corner_co

    if next_chain.vert_cos:
        return next_chain.start_loop_index, next_chain.start_vert_index, next_chain.vert_cos[0].copy()
    if prev_chain.vert_cos:
        return prev_chain.end_loop_index, prev_chain.end_vert_index, prev_chain.vert_cos[-1].copy()
    return -1, -1, Vector((0.0, 0.0, 0.0))


def _build_loop_corners(boundary_loop, basis_u, basis_v):
    """Build loop corners from final chain junctions or geometric turns."""

    chain_count = len(boundary_loop.chains)
    if chain_count < 2:
        return _build_geometric_loop_corners(boundary_loop, basis_u, basis_v)

    corners = []
    for next_chain_index in range(chain_count):
        prev_chain_index = (next_chain_index - 1) % chain_count
        prev_chain = boundary_loop.chains[prev_chain_index]
        next_chain = boundary_loop.chains[next_chain_index]
        loop_vert_index, vert_index, corner_co = _resolve_loop_corner_from_final_chains(
            boundary_loop,
            prev_chain,
            next_chain,
        )

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
        loop_kind = raw_loop.kind
        if not isinstance(loop_kind, LoopKind):
            loop_kind = LoopKind(loop_kind)

        boundary_loop = BoundaryLoop(
            vert_indices=list(raw_loop.vert_indices),
            vert_cos=[co.copy() for co in raw_loop.vert_cos],
            edge_indices=list(raw_loop.edge_indices),
            side_face_indices=list(raw_loop.side_face_indices),
            kind=loop_kind,
            depth=int(raw_loop.depth),
        )

        loop_neighbors = [
            _neighbor_for_side(edge_index, side_face_index, patch_face_indices, face_to_patch, patch_id, bm)
            for edge_index, side_face_index in zip(raw_loop.edge_indices, raw_loop.side_face_indices)
        ]
        raw_chains = _split_loop_into_chains_by_neighbor(
            raw_loop.vert_indices,
            raw_loop.vert_cos,
            raw_loop.edge_indices,
            raw_loop.side_face_indices,
            loop_neighbors,
        )
        # Мержим chains, разделённые бевель-заворотом: поворот вдоль нормали,
        # а не в касательной плоскости. На UV это один непрерывный сегмент.
        raw_chains = _merge_bevel_wrap_chains(raw_chains, bm)

        raw_chains = _try_geometric_outer_loop_split(raw_loop, raw_chains, basis_u, basis_v, bm=bm)
        raw_chains = _split_border_chains_by_corners(raw_chains, basis_u, basis_v, bm=bm)
        boundary_loop.chains = _build_boundary_chain_objects(raw_chains, basis_u, basis_v)
        _downgrade_same_role_point_contact_chains(boundary_loop.chains, basis_u, basis_v, patch_id, loop_index)
        boundary_loop.chains = _merge_same_role_border_chains(boundary_loop.chains)

        boundary_loop.corners = _build_loop_corners(boundary_loop, basis_u, basis_v)
        _assign_loop_chain_endpoint_topology(boundary_loop)
        boundary_loops.append(boundary_loop)

    return boundary_loops


def _classify_loops_outer_hole(bm, raw_patch_data, obj=None):
    """Classify loops as OUTER or HOLE through the single explicit UV-dependent analysis boundary."""

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

    raw_patch_data = {}
    patches_raw = _flood_fill_patches(bm, face_indices)

    for patch_id, patch_face_indices in enumerate(patches_raw):
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
        patch_graph.add_node(node)
        raw_patch_data[patch_id] = _RawPatchBoundaryData(
            face_indices=list(patch_face_indices),
            raw_loops=raw_loops,
        )

    # Единственный допустимый side effect внутри analysis: временный UV unwrap
    # для определения OUTER/HOLE у multi-loop boundary.
    _classify_loops_outer_hole(bm, raw_patch_data, obj)

    for patch_id, node in patch_graph.nodes.items():
        patch_data = raw_patch_data[patch_id]
        node.boundary_loops = _build_boundary_loops(
            patch_data.raw_loops,
            patch_data.face_indices,
            patch_graph.face_to_patch,
            patch_id,
            node.basis_u,
            node.basis_v,
            bm,
        )

    for seam_edge in _build_seam_edges(patch_graph.face_to_patch, bm):
        patch_graph.add_edge(seam_edge)

    return patch_graph


def _chain_length(chain):
    """Compute a polyline length for the debug report."""

    if len(chain.vert_cos) < 2:
        return 0.0

    length = 0.0
    for index in range(len(chain.vert_cos) - 1):
        length += (chain.vert_cos[index + 1] - chain.vert_cos[index]).length
    return length


def _format_chain_refs(chain_refs):
    if not chain_refs:
        return "[]"
    return "[" + " ".join(f"L{loop_index}C{chain_index}" for loop_index, chain_index in chain_refs) + "]"


def _format_frame_run_chain_indices(chain_indices):
    if not chain_indices:
        return "[]"
    return "[" + " ".join(f"C{chain_index}" for chain_index in chain_indices) + "]"


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


@dataclass
class _Junction:
    vert_index: int
    vert_co: Vector
    corner_refs: list[_JunctionCornerRef] = field(default_factory=list)
    chain_refs: list[_JunctionChainRef] = field(default_factory=list)
    run_endpoint_refs: list[_JunctionRunEndpointRef] = field(default_factory=list)
    patch_ids: list[int] = field(default_factory=list)
    valence: int = 0
    has_mesh_border: bool = False
    has_seam_self: bool = False
    is_open: bool = False
    h_count: int = 0
    v_count: int = 0
    free_count: int = 0


@dataclass
class _PatchGraphDiagnostics:
    frame_runs_by_loop: dict[PatchLoopKey, list[_FrameRun]] = field(default_factory=dict)
    junctions: list[_Junction] = field(default_factory=list)
    interesting_junctions: list[_Junction] = field(default_factory=list)
    open_junction_count: int = 0
    valence_histogram: dict[int, int] = field(default_factory=dict)


def _build_patch_graph_junctions(graph, frame_runs_by_loop=None):
    """Build a diagnostic-only junction view from final patch/loop/corner topology."""

    junction_entries = {}
    run_refs_by_corner: dict[CornerJunctionKey, list[_JunctionRunEndpointRef]] = {}

    if frame_runs_by_loop:
        for (patch_id, loop_index), frame_runs in frame_runs_by_loop.items():
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

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for corner_index, corner in enumerate(boundary_loop.corners):
                if corner.vert_index < 0:
                    continue

                entry = junction_entries.setdefault(
                    corner.vert_index,
                    {
                        "vert_co": corner.vert_co.copy(),
                        "corner_refs": [],
                        "chain_refs": {},
                        "patch_ids": set(),
                    },
                )
                entry["patch_ids"].add(patch_id)
                entry["corner_refs"].append(
                    _JunctionCornerRef(
                        patch_id=patch_id,
                        loop_index=loop_index,
                        corner_index=corner_index,
                        prev_chain_index=corner.prev_chain_index,
                        next_chain_index=corner.next_chain_index,
                    )
                )
                for run_ref in run_refs_by_corner.get((patch_id, loop_index, corner_index), []):
                    entry.setdefault("run_endpoint_refs", []).append(run_ref)

                for chain_index in (corner.prev_chain_index, corner.next_chain_index):
                    chain = graph.get_chain(patch_id, loop_index, chain_index)
                    if chain is None:
                        continue
                    chain_key = (patch_id, loop_index, chain_index)
                    if chain_key in entry["chain_refs"]:
                        continue
                    entry["chain_refs"][chain_key] = _JunctionChainRef(
                        patch_id=patch_id,
                        loop_index=loop_index,
                        chain_index=chain_index,
                        frame_role=chain.frame_role,
                        neighbor_kind=chain.neighbor_kind,
                    )

    junctions = []
    for vert_index in sorted(junction_entries.keys()):
        entry = junction_entries[vert_index]
        chain_refs = list(entry["chain_refs"].values())
        corner_refs = list(entry["corner_refs"])
        run_endpoint_refs = list(entry.get("run_endpoint_refs", []))
        patch_ids = sorted(entry["patch_ids"])

        h_count = sum(1 for chain_ref in chain_refs if chain_ref.frame_role == FrameRole.H_FRAME)
        v_count = sum(1 for chain_ref in chain_refs if chain_ref.frame_role == FrameRole.V_FRAME)
        free_count = sum(1 for chain_ref in chain_refs if chain_ref.frame_role == FrameRole.FREE)
        has_mesh_border = any(
            chain_ref.neighbor_kind == ChainNeighborKind.MESH_BORDER for chain_ref in chain_refs
        )
        has_seam_self = any(
            chain_ref.neighbor_kind == ChainNeighborKind.SEAM_SELF for chain_ref in chain_refs
        )

        junctions.append(
            _Junction(
                vert_index=vert_index,
                vert_co=entry["vert_co"].copy(),
                corner_refs=sorted(
                    corner_refs,
                    key=lambda ref: (ref.patch_id, ref.loop_index, ref.corner_index),
                ),
                chain_refs=sorted(
                    chain_refs,
                    key=lambda ref: (ref.patch_id, ref.loop_index, ref.chain_index),
                ),
                run_endpoint_refs=sorted(
                    run_endpoint_refs,
                    key=lambda ref: (ref.patch_id, ref.loop_index, ref.run_index, ref.endpoint_kind),
                ),
                patch_ids=patch_ids,
                valence=len(corner_refs),
                has_mesh_border=has_mesh_border,
                has_seam_self=has_seam_self,
                is_open=has_mesh_border,
                h_count=h_count,
                v_count=v_count,
                free_count=free_count,
            )
        )

    return junctions


def _format_junction_corner_refs(corner_refs):
    if not corner_refs:
        return "[]"
    return "[" + " ".join(
        f"P{corner_ref.patch_id}L{corner_ref.loop_index}K{corner_ref.corner_index}"
        for corner_ref in corner_refs
    ) + "]"


def _format_junction_chain_refs(chain_refs):
    if not chain_refs:
        return "[]"
    return "[" + " ".join(
        f"P{chain_ref.patch_id}L{chain_ref.loop_index}C{chain_ref.chain_index}"
        for chain_ref in chain_refs
    ) + "]"


def _format_junction_run_endpoint_refs(run_endpoint_refs):
    if not run_endpoint_refs:
        return "[]"
    return "[" + " ".join(
        f"P{run_ref.patch_id}L{run_ref.loop_index}R{run_ref.run_index}:{run_ref.endpoint_kind}"
        for run_ref in run_endpoint_refs
    ) + "]"


def _build_patch_graph_diagnostics(graph):
    """Build the derived diagnostic bundle used by PatchGraph console reporting."""

    frame_runs_by_loop = _build_patch_graph_frame_runs(graph)
    junctions = _build_patch_graph_junctions(graph, frame_runs_by_loop=frame_runs_by_loop)
    interesting_junctions = sorted(
        [
            junction
            for junction in junctions
            if junction.valence >= 2 or junction.has_seam_self
        ],
        key=lambda junction: (
            0 if not junction.is_open else 1,
            -junction.valence,
            junction.vert_index,
        ),
    )

    valence_histogram = {}
    open_junction_count = 0
    for junction in junctions:
        valence_histogram[junction.valence] = valence_histogram.get(junction.valence, 0) + 1
        if junction.is_open:
            open_junction_count += 1

    return _PatchGraphDiagnostics(
        frame_runs_by_loop=frame_runs_by_loop,
        junctions=junctions,
        interesting_junctions=interesting_junctions,
        open_junction_count=open_junction_count,
        valence_histogram=valence_histogram,
    )


def format_patch_graph_report(graph, mesh_name=None) -> FormattedReport:
    """Build text lines for the System Console PatchGraph report."""

    lines = []
    diagnostics = _build_patch_graph_diagnostics(graph)

    total_patches = len(graph.nodes)
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
    total_mesh_borders = 0
    total_self_seams = 0

    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        patch_type = _enum_value(node.patch_type)
        world_facing = _enum_value(node.world_facing)
        if world_facing == WorldFacing.UP.value:
            total_up += 1
        elif world_facing == WorldFacing.DOWN.value:
            total_down += 1
        else:
            total_side += 1

        if patch_type == PatchType.WALL.value:
            walls += 1
        elif patch_type == PatchType.FLOOR.value:
            floors += 1
        elif patch_type == PatchType.SLOPE.value:
            slopes += 1

        if len(node.face_indices) == 1:
            singles += 1

        chain_count = 0
        corner_count = 0
        patch_roles = []
        loop_kinds = []
        loop_details = []

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            total_loops += 1
            loop_kind = _enum_value(boundary_loop.kind)
            loop_kinds.append(loop_kind)
            if loop_kind == LoopKind.HOLE.value:
                total_holes += 1

            loop_corner_count = len(boundary_loop.corners)
            chain_count += len(boundary_loop.chains)
            corner_count += loop_corner_count
            loop_details.append(
                f"    Loop {loop_index}: {loop_kind} | chains:{len(boundary_loop.chains)} corners:{loop_corner_count}"
            )

            for chain_index, chain in enumerate(boundary_loop.chains):
                role = _enum_value(chain.frame_role)
                neighbor_kind = _enum_value(chain.neighbor_kind)
                neighbor_suffix = ""

                if role == FrameRole.H_FRAME.value:
                    total_h += 1
                elif role == FrameRole.V_FRAME.value:
                    total_v += 1
                else:
                    total_free += 1

                if neighbor_kind == ChainNeighborKind.PATCH.value:
                    total_patch_links += 1
                    neighbor_node = graph.nodes.get(chain.neighbor_patch_id)
                    neighbor_semantic = graph.get_patch_semantic_key(chain.neighbor_patch_id) if neighbor_node else "UNKNOWN"
                    neighbor_suffix = f" -> Patch {chain.neighbor_patch_id}:{neighbor_semantic}"
                elif neighbor_kind == ChainNeighborKind.SEAM_SELF.value:
                    total_self_seams += 1
                else:
                    total_mesh_borders += 1

                transition = graph.describe_chain_transition(patch_id, chain)
                endpoint_neighbors = graph.get_chain_endpoint_neighbors(patch_id, loop_index, chain_index)
                start_corner = chain.start_corner_index if chain.start_corner_index >= 0 else "-"
                end_corner = chain.end_corner_index if chain.end_corner_index >= 0 else "-"
                bridge_tag = " [BRIDGE]" if role == FrameRole.FREE.value and len(chain.vert_cos) <= 2 else ""
                patch_roles.append(role)
                loop_details.append(
                    f"      Chain {chain_index}: {role}{bridge_tag} | neighbor:{neighbor_kind}{neighbor_suffix} | transition:{transition} | "
                    f"verts:{chain.start_vert_index}->{chain.end_vert_index} | corners:{start_corner}->{end_corner} | "
                    f"ep:start{_format_chain_refs(endpoint_neighbors['start'])} end{_format_chain_refs(endpoint_neighbors['end'])} | "
                    f"edges:{len(chain.edge_indices)} | length:{_chain_length(chain):.4f}"
                )

            frame_runs = diagnostics.frame_runs_by_loop.get((patch_id, loop_index), [])
            for run_index, frame_run in enumerate(frame_runs):
                start_corner = frame_run.start_corner_index if frame_run.start_corner_index >= 0 else "-"
                end_corner = frame_run.end_corner_index if frame_run.end_corner_index >= 0 else "-"
                loop_details.append(
                    f"      Run {run_index}: {frame_run.dominant_role.value} | "
                    f"chains:{_format_frame_run_chain_indices(frame_run.chain_indices)} | "
                    f"corners:{start_corner}->{end_corner} | "
                    f"total:{frame_run.total_length:.4f} support:{frame_run.support_length:.4f} "
                    f"free_gap:{frame_run.gap_free_length:.4f} max_gap:{frame_run.max_free_gap_length:.4f} | "
                    f"span:u={frame_run.projected_u_span:.4f} v={frame_run.projected_v_span:.4f}"
                )

            for corner_index, corner in enumerate(boundary_loop.corners):
                total_corners += 1
                is_sharp = corner.turn_angle_deg >= CORNER_ANGLE_THRESHOLD_DEG
                if is_sharp:
                    total_sharp_corners += 1

                loop_details.append(
                    f"      Corner {corner_index}: {corner.corner_type} | chains:{corner.prev_chain_index}->{corner.next_chain_index} | "
                    f"vert:{corner.vert_index} | turn:{corner.turn_angle_deg:.1f} | sharp:{'Y' if is_sharp else 'N'}"
                )

        total_chains += chain_count
        lines.append(
            f"  Patch {patch_id}: {patch_type} | facing:{world_facing} | {len(node.face_indices)}f | "
            f"loops:{len(node.boundary_loops)}[{' '.join(loop_kinds)}] chains:{chain_count} corners:{corner_count} | "
            f"roles:[{' '.join(patch_roles)}]"
        )
        lines.extend(loop_details)

    if diagnostics.junctions:
        valence_summary = " ".join(
            f"v{valence}:{count}" for valence, count in sorted(diagnostics.valence_histogram.items())
        )
        lines.append(
            f"  Junctions: total:{len(diagnostics.junctions)} interesting:{len(diagnostics.interesting_junctions)} "
            f"open:{diagnostics.open_junction_count} closed:{len(diagnostics.junctions) - diagnostics.open_junction_count} "
            f"valence:[{valence_summary}]"
        )
        for junction in diagnostics.interesting_junctions:
            patch_refs = "[" + " ".join(f"P{patch_id}" for patch_id in junction.patch_ids) + "]"
            lines.append(
                f"    Junction V{junction.vert_index}: valence:{junction.valence} patches:{patch_refs} "
                f"open:{'Y' if junction.is_open else 'N'} border:{'Y' if junction.has_mesh_border else 'N'} "
                f"self:{'Y' if junction.has_seam_self else 'N'} | "
                f"roles:H{junction.h_count} V{junction.v_count} F{junction.free_count} | "
                f"corners:{_format_junction_corner_refs(junction.corner_refs)} | "
                f"chains:{_format_junction_chain_refs(junction.chain_refs)} | "
                f"runs:{_format_junction_run_endpoint_refs(junction.run_endpoint_refs)}"
            )

    summary = (
        f"Patches: {total_patches} (W:{walls} F:{floors} S:{slopes} 1f:{singles}) | "
        f"Loops: {total_loops} Chains: {total_chains} Corners: {total_corners} Sharp:{total_sharp_corners} Holes: {total_holes} | "
        f"Roles: H:{total_h} V:{total_v} Free:{total_free} | "
        f"Facing: Up:{total_up} Down:{total_down} Side:{total_side} | "
        f"Neighbors: Patch:{total_patch_links} Self:{total_self_seams} Border:{total_mesh_borders} | "
        f"Junctions: {len(diagnostics.junctions)} Interesting:{len(diagnostics.interesting_junctions)}"
    )

    return FormattedReport(lines=lines, summary=summary)


def format_patch_graph_snapshot_report(graph, mesh_name=None) -> FormattedReport:
    """Build a compact, stable PatchGraph snapshot for regression baselines."""

    lines = []
    diagnostics = _build_patch_graph_diagnostics(graph)

    total_loops = 0
    total_chains = 0
    total_corners = 0
    total_holes = 0
    total_h = 0
    total_v = 0
    total_free = 0
    total_patch_links = 0
    total_mesh_borders = 0
    total_self_seams = 0
    total_run_h = 0
    total_run_v = 0
    total_run_free = 0

    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        patch_h = 0
        patch_v = 0
        patch_free = 0
        patch_chain_count = 0
        patch_corner_count = 0
        patch_holes = 0
        patch_run_count = 0

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            total_loops += 1
            patch_chain_count += len(boundary_loop.chains)
            patch_corner_count += len(boundary_loop.corners)
            total_chains += len(boundary_loop.chains)
            total_corners += len(boundary_loop.corners)
            if boundary_loop.kind == LoopKind.HOLE:
                total_holes += 1
                patch_holes += 1

            for chain in boundary_loop.chains:
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

            frame_runs = diagnostics.frame_runs_by_loop.get((patch_id, loop_index), [])
            patch_run_count += len(frame_runs)
            for frame_run in frame_runs:
                if frame_run.dominant_role == FrameRole.H_FRAME:
                    total_run_h += 1
                elif frame_run.dominant_role == FrameRole.V_FRAME:
                    total_run_v += 1
                else:
                    total_run_free += 1

        lines.append(
            f"P{patch_id} {graph.get_patch_semantic_key(patch_id)} | "
            f"faces:{len(node.face_indices)} loops:{len(node.boundary_loops)} holes:{patch_holes} "
            f"chains:{patch_chain_count} corners:{patch_corner_count} runs:{patch_run_count} | "
            f"roles:H{patch_h} V{patch_v} F{patch_free}"
        )

    valence_summary = " ".join(
        f"v{valence}:{count}" for valence, count in sorted(diagnostics.valence_histogram.items())
    )
    lines.append(
        "Topology: "
        f"patches:{len(graph.nodes)} loops:{total_loops} chains:{total_chains} corners:{total_corners} holes:{total_holes}"
    )
    lines.append(
        "Roles: "
        f"H:{total_h} V:{total_v} Free:{total_free} | "
        f"Runs: H:{total_run_h} V:{total_run_v} Free:{total_run_free}"
    )
    lines.append(
        "Neighbors: "
        f"Patch:{total_patch_links} Self:{total_self_seams} Border:{total_mesh_borders}"
    )
    lines.append(
        "Junctions: "
        f"total:{len(diagnostics.junctions)} interesting:{len(diagnostics.interesting_junctions)} "
        f"open:{diagnostics.open_junction_count} "
        f"closed:{len(diagnostics.junctions) - diagnostics.open_junction_count} "
        f"valence:[{valence_summary}]"
    )

    summary = (
        f"PatchGraph snapshot | patches:{len(graph.nodes)} loops:{total_loops} chains:{total_chains} "
        f"corners:{total_corners} runs:{total_run_h + total_run_v + total_run_free} "
        f"junctions:{len(diagnostics.junctions)}"
    )
    return FormattedReport(lines=lines, summary=summary)





