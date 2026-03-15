from __future__ import annotations

import math

import bmesh
import bpy
from mathutils import Vector

try:
    from .constants import (
        CORNER_ANGLE_THRESHOLD_DEG,
        FLOOR_THRESHOLD,
        FRAME_ALIGNMENT_THRESHOLD,
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
    )
except ImportError:
    from constants import (
        CORNER_ANGLE_THRESHOLD_DEG,
        FLOOR_THRESHOLD,
        FRAME_ALIGNMENT_THRESHOLD,
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
    )


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


def format_solver_input_preflight_report(report, mesh_name=None):
    """Build text lines for blocking solve preflight issues."""

    lines = []
    if mesh_name:
        lines.append(f'Mesh: {mesh_name}')
    lines.append(f'Checked faces: {len(report.checked_face_indices)}')

    if report.is_valid:
        summary = f'Solver preflight: OK | faces:{len(report.checked_face_indices)}'
        return lines, summary

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
    return lines, summary

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

    edge_dirs = {}
    for face in patch_faces:
        for edge in face.edges:
            length = edge.calc_length()
            if length < 1e-4:
                continue
            vec = (edge.verts[1].co - edge.verts[0].co).normalized()
            if vec.dot(WORLD_UP) < 0.0:
                vec = -vec
            key = (round(vec.x, 2), round(vec.y, 2), round(vec.z, 2))
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
    return (loop.face.index, loop.edge.index, loop.vert.index)


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
                    {
                        "vert_indices": vert_indices,
                        "vert_cos": vert_cos,
                        "edge_indices": edge_indices,
                        "side_face_indices": side_face_indices,
                        "kind": LoopKind.OUTER,
                        "depth": 0,
                        "closed": closed,
                    }
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
            {
                "vert_indices": list(loop_vert_indices),
                "vert_cos": list(loop_vert_cos),
                "edge_indices": list(loop_edge_indices),
                "side_face_indices": list(loop_side_face_indices),
                "neighbor": neighbors[0],
                "is_closed": True,
                "start_loop_index": 0,
                "end_loop_index": 0,
            }
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
            {
                "vert_indices": chain_vert_indices,
                "vert_cos": chain_vert_cos,
                "edge_indices": chain_edge_indices,
                "side_face_indices": chain_side_face_indices,
                "neighbor": neighbors[v_start % edge_count],
                "is_closed": False,
                "start_loop_index": v_start % vertex_count,
                "end_loop_index": v_end % vertex_count,
            }
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
        cos_a = chain_a.get("vert_cos", [])
        cos_b = chain_b.get("vert_cos", [])
        if len(cos_a) < 2 or len(cos_b) < 2:
            return False

        # Точка стыка = последняя вершина chain_a = первая вершина chain_b
        corner_co = cos_a[-1]
        prev_point = cos_a[-2]
        next_point = cos_b[1] if len(cos_b) > 1 else cos_b[0]

        # Vertex normal в точке стыка
        vert_index = chain_a["vert_indices"][-1]
        if vert_index >= len(bm.verts):
            return False
        vert_normal = bm.verts[vert_index].normal
        if vert_normal.length_squared < 1e-12:
            return False

        return not _is_tangent_plane_turn(corner_co, prev_point, next_point, vert_normal)

    def _do_merge(chain_a, chain_b):
        """Объединяет chain_b в chain_a (chain_b's first vertex = chain_a's last)."""
        return {
            "vert_indices": chain_a["vert_indices"] + chain_b["vert_indices"][1:],
            "vert_cos": chain_a["vert_cos"] + chain_b["vert_cos"][1:],
            "edge_indices": chain_a["edge_indices"] + chain_b["edge_indices"],
            "side_face_indices": chain_a["side_face_indices"] + chain_b["side_face_indices"][1:],
            "neighbor": chain_a.get("neighbor", NB_MESH_BORDER),
            "is_closed": chain_a.get("is_closed", False),
            "start_loop_index": chain_a.get("start_loop_index", 0),
            "end_loop_index": chain_b.get("end_loop_index", 0),
        }

    # Итеративно мержим пока есть что мержить
    merged = True
    result = list(chains)
    while merged:
        merged = False
        new_result = []
        i = 0
        while i < len(result):
            if i + 1 < len(result) and _should_merge(result[i], result[i + 1]):
                vi = result[i]["vert_indices"][-1]
                print(f"  [BevelMerge] merging chains at vert {vi}: "
                      f"{len(result[i]['vert_indices'])}v + {len(result[i+1]['vert_indices'])}v")
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
        raw_loops[0]["kind"] = LoopKind.OUTER
        raw_loops[0]["depth"] = 0
        return

    patch_face_indices = set(patch_face_indices)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    def get_side_uv(face_index, edge_index, vert_index):
        face = bm.faces[face_index]
        for loop in face.loops:
            if loop.edge.index == edge_index and loop.vert.index == vert_index:
                uv = loop[uv_layer].uv
                return (uv.x, uv.y)

        vert = bm.verts[vert_index]
        uvs = [loop[uv_layer].uv for loop in vert.link_loops if loop.face.index in patch_face_indices]
        if not uvs:
            return (0.0, 0.0)
        return (
            sum(uv.x for uv in uvs) / len(uvs),
            sum(uv.y for uv in uvs) / len(uvs),
        )

    polys_2d = []
    for raw_loop in raw_loops:
        poly = []
        for face_index, edge_index, vert_index in zip(
            raw_loop["side_face_indices"],
            raw_loop["edge_indices"],
            raw_loop["vert_indices"],
        ):
            poly.append(get_side_uv(face_index, edge_index, vert_index))
        polys_2d.append(poly)

    def signed_area(poly):
        area = 0.0
        count = len(poly)
        for idx in range(count):
            x1, y1 = poly[idx]
            x2, y2 = poly[(idx + 1) % count]
            area += x1 * y2 - x2 * y1
        return 0.5 * area

    def point_in_poly(point, poly):
        x, y = point
        inside = False
        count = len(poly)
        for idx in range(count):
            x1, y1 = poly[idx]
            x2, y2 = poly[(idx + 1) % count]
            if (y1 > y) != (y2 > y):
                x_intersection = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-30) + x1
                if x < x_intersection:
                    inside = not inside
        return inside

    def interior_point(poly):
        count = len(poly)
        if count < 3:
            return poly[0] if poly else (0.0, 0.0)

        signed = signed_area(poly)
        edges_by_len = []
        for idx in range(count):
            next_idx = (idx + 1) % count
            dx = poly[next_idx][0] - poly[idx][0]
            dy = poly[next_idx][1] - poly[idx][1]
            edges_by_len.append((dx * dx + dy * dy, idx))
        edges_by_len.sort(reverse=True)

        for len_squared, idx in edges_by_len:
            next_idx = (idx + 1) % count
            mid_x = (poly[idx][0] + poly[next_idx][0]) * 0.5
            mid_y = (poly[idx][1] + poly[next_idx][1]) * 0.5
            dx = poly[next_idx][0] - poly[idx][0]
            dy = poly[next_idx][1] - poly[idx][1]
            edge_len = math.sqrt(len_squared)
            if edge_len < 1e-12:
                continue

            if signed >= 0.0:
                normal_x, normal_y = -dy / edge_len, dx / edge_len
            else:
                normal_x, normal_y = dy / edge_len, -dx / edge_len

            epsilon = edge_len * 0.01
            point = (mid_x + normal_x * epsilon, mid_y + normal_y * epsilon)
            if point_in_poly(point, poly):
                return point

        return (
            sum(point[0] for point in poly) / count,
            sum(point[1] for point in poly) / count,
        )

    interior_points = [interior_point(poly) for poly in polys_2d]

    for loop_index, raw_loop in enumerate(raw_loops):
        depth = 0
        for poly_index, poly in enumerate(polys_2d):
            if loop_index == poly_index:
                continue
            if point_in_poly(interior_points[loop_index], poly):
                depth += 1

        raw_loop["depth"] = depth
        raw_loop["kind"] = LoopKind.OUTER if depth == 0 or depth % 2 == 0 else LoopKind.HOLE


def _classify_chain_frame_role(chain_vert_cos, basis_u, basis_v, threshold=FRAME_ALIGNMENT_THRESHOLD, strict_guards=True):
    """Classify a boundary chain against the local patch basis.

    strict_guards=True  — применяет waviness/curvature guards (для PATCH/SEAM chains)
    strict_guards=False — только ratio check, как в legacy (для MESH_BORDER chains)
    """

    if len(chain_vert_cos) < 2:
        return FrameRole.FREE

    points_2d = [Vector((vert_co.dot(basis_u), vert_co.dot(basis_v))) for vert_co in chain_vert_cos]
    us = [point.x for point in points_2d]
    vs = [point.y for point in points_2d]

    extent_u = max(us) - min(us)
    extent_v = max(vs) - min(vs)
    total_extent = max(extent_u, extent_v)
    if total_extent < 1e-6:
        return FrameRole.FREE

    ratio_v = extent_v / total_extent
    ratio_u = extent_u / total_extent

    # MESH_BORDER chains: только ratio check, без waviness/curvature guards.
    # Архитектурная геометрия (бевели, ступеньки) не должна отвергать H/V.
    if not strict_guards:
        if ratio_v < threshold:
            return FrameRole.H_FRAME
        if ratio_u < threshold:
            return FrameRole.V_FRAME
        return FrameRole.FREE

    # PATCH/SEAM chains: ratio + ослабленные waviness/curvature guards.
    polyline_length = 0.0
    for point_index in range(len(points_2d) - 1):
        polyline_length += (points_2d[point_index + 1] - points_2d[point_index]).length
    if polyline_length < 1e-6:
        return FrameRole.FREE

    def _is_wavy_axis(primary_axis: str) -> bool:
        if len(points_2d) < 4:
            return False

        if primary_axis == 'v':
            primary_span = extent_v
            orth_extent = extent_u
        else:
            primary_span = extent_u
            orth_extent = extent_v

        if primary_span < 1e-6:
            return True

        orth_deltas = []
        min_step = max(primary_span * 0.005, 1e-5)
        orthogonal_travel = 0.0
        for point_index in range(len(points_2d) - 1):
            delta = points_2d[point_index + 1] - points_2d[point_index]
            orth_step = delta.x if primary_axis == 'v' else delta.y
            if abs(orth_step) < min_step:
                continue
            orthogonal_travel += abs(orth_step)
            orth_deltas.append(orth_step)

        if len(orth_deltas) < 3:
            return False

        sign_changes = 0
        prev_sign = 0
        for orth_step in orth_deltas:
            sign = 1 if orth_step > 0.0 else -1
            if prev_sign != 0 and sign != prev_sign:
                sign_changes += 1
            prev_sign = sign

        if sign_changes < 2:
            return False

        # Ослабленные thresholds: 0.25 (было 0.12), 3.0 (было 2.0)
        # Архитектурные бевели дают orth travel ~15-20% span, что ложно
        # срабатывало на старых thresholds
        return orthogonal_travel > max(primary_span * 0.25, orth_extent * 3.0)

    def _is_curved_against_chord(primary_axis: str) -> bool:
        chord = points_2d[-1] - points_2d[0]
        chord_length = chord.length
        if chord_length < 1e-6:
            return True

        if primary_axis == 'v':
            primary_span = extent_v
            orth_extent = extent_u
        else:
            primary_span = extent_u
            orth_extent = extent_v

        length_excess = (polyline_length / chord_length) - 1.0
        # Ослабленный порог: 0.05 (было 0.015)
        # Бевельные переходы дают excess ~2-4%, что ложно блокировало H/V
        if length_excess <= 0.05:
            return False

        chord_dir = chord / chord_length
        max_chord_deviation = 0.0
        prev_dir = None
        turn_budget_deg = 0.0
        for point_index in range(1, len(points_2d) - 1):
            rel = points_2d[point_index] - points_2d[0]
            proj_len = rel.dot(chord_dir)
            closest = points_2d[0] + chord_dir * proj_len
            max_chord_deviation = max(max_chord_deviation, (points_2d[point_index] - closest).length)

        for point_index in range(len(points_2d) - 1):
            delta = points_2d[point_index + 1] - points_2d[point_index]
            if delta.length <= 1e-8:
                continue
            direction = delta.normalized()
            if prev_dir is not None:
                turn_budget_deg += abs(math.degrees(prev_dir.angle(direction, 0.0)))
            prev_dir = direction

        # Ослабленные thresholds: 0.08 (было 0.03), 1.0 (было 0.75), 0.02 (было 0.01)
        deviation_limit = max(primary_span * 0.08, orth_extent * 1.0, 0.02)
        # Ослабленный turn_budget: 60.0 (было 35.0)
        return max_chord_deviation > deviation_limit and (length_excess > 0.08 or turn_budget_deg > 60.0)

    if ratio_v < threshold and not _is_wavy_axis('h') and not _is_curved_against_chord('h'):
        return FrameRole.H_FRAME
    if ratio_u < threshold and not _is_wavy_axis('v') and not _is_curved_against_chord('v'):
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
    """Find stable corner indices for a closed OUTER loop."""

    vertex_count = len(loop_vert_cos)
    if vertex_count < 4:
        return []

    # Подготовка vertex normals для фильтрации бевель-заворотов
    # Vertex normal (BMVert.normal) — усреднённая по всем смежным faces,
    # стабильна на стыке разно-ориентированных граней (бевель-угол).
    use_vert_normals = (loop_vert_indices is not None and bm is not None)
    if use_vert_normals:
        bm.verts.ensure_lookup_table()

    candidate_corners = []
    for loop_vert_index in range(vertex_count):
        corner_co = loop_vert_cos[loop_vert_index]
        prev_point = loop_vert_cos[(loop_vert_index - 1) % vertex_count]
        next_point = loop_vert_cos[(loop_vert_index + 1) % vertex_count]
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)
        if turn_angle_deg < CORNER_ANGLE_THRESHOLD_DEG:
            continue
        # Фильтр: пропускаем бевель-завороты (поворот вдоль нормали)
        if use_vert_normals:
            vi = loop_vert_indices[loop_vert_index]
            if vi < len(bm.verts):
                vn = bm.verts[vi].normal
                if vn.length_squared > 1e-12 and not _is_tangent_plane_turn(corner_co, prev_point, next_point, vn):
                    continue
        candidate_corners.append((loop_vert_index, turn_angle_deg))

    if len(candidate_corners) < 4:
        return []

    perimeter = _loop_arc_length(loop_vert_cos, 0, 0)
    min_span_length = max(perimeter * 0.04, 1e-4)
    min_vertex_gap = 2

    filtered_corners = []
    for loop_vert_index, turn_angle_deg in candidate_corners:
        if not filtered_corners:
            filtered_corners.append((loop_vert_index, turn_angle_deg))
            continue

        prev_loop_index, prev_turn_angle_deg = filtered_corners[-1]
        span_length = _loop_arc_length(loop_vert_cos, prev_loop_index, loop_vert_index)
        span_vertex_count = (loop_vert_index - prev_loop_index) % vertex_count
        if span_vertex_count < min_vertex_gap or span_length < min_span_length:
            if turn_angle_deg > prev_turn_angle_deg:
                filtered_corners[-1] = (loop_vert_index, turn_angle_deg)
            continue

        filtered_corners.append((loop_vert_index, turn_angle_deg))

    while len(filtered_corners) >= 2:
        first_loop_index, first_turn_angle_deg = filtered_corners[0]
        last_loop_index, last_turn_angle_deg = filtered_corners[-1]
        wrap_span_length = _loop_arc_length(loop_vert_cos, last_loop_index, first_loop_index)
        wrap_span_vertex_count = (first_loop_index - last_loop_index) % vertex_count
        if wrap_span_vertex_count >= min_vertex_gap and wrap_span_length >= min_span_length:
            break

        if first_turn_angle_deg >= last_turn_angle_deg:
            filtered_corners.pop()
        else:
            filtered_corners.pop(0)

    if len(filtered_corners) < 4:
        return []

    return [loop_vert_index for loop_vert_index, _ in filtered_corners]


def _split_closed_loop_by_corner_indices(raw_loop, split_indices, neighbor_patch_id):
    """Split one closed raw loop into raw chains at geometric corners."""

    loop_vert_indices = list(raw_loop.get("vert_indices", []))
    loop_vert_cos = list(raw_loop.get("vert_cos", []))
    loop_edge_indices = list(raw_loop.get("edge_indices", []))
    loop_side_face_indices = list(raw_loop.get("side_face_indices", []))
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
            {
                "vert_indices": chain_vert_indices,
                "vert_cos": chain_vert_cos,
                "edge_indices": chain_edge_indices,
                "side_face_indices": chain_side_face_indices,
                "neighbor": neighbor_patch_id,
                "is_closed": False,
                "start_loop_index": start_loop_index,
                "end_loop_index": end_loop_index,
            }
        )

    return raw_chains


def _find_open_chain_corners(vert_cos, basis_u, basis_v, min_spacing=2,
                             vert_indices=None, bm=None):
    """Detect corners on an open (non-closed) chain by per-vertex angle.

    Returns list of indices into vert_cos where turn angle >= CORNER_ANGLE_THRESHOLD_DEG.
    Excludes endpoints (0 and len-1) — they are chain boundaries, not interior corners.
    Бевель-завороты (поворот вдоль vertex normal) отфильтровываются если переданы
    vert_indices и bm.
    """
    vertex_count = len(vert_cos)
    if vertex_count < 3:
        return []

    use_vert_normals = (vert_indices is not None and bm is not None)

    candidates = []
    for i in range(1, vertex_count - 1):
        turn_angle = _measure_corner_turn_angle(vert_cos[i], vert_cos[i - 1], vert_cos[i + 1], basis_u, basis_v)
        if turn_angle >= CORNER_ANGLE_THRESHOLD_DEG:
            # Фильтр: бевель-завороты
            if use_vert_normals:
                vi = vert_indices[i]
                if vi < len(bm.verts):
                    vn = bm.verts[vi].normal
                    if vn.length_squared > 1e-12 and not _is_tangent_plane_turn(
                            vert_cos[i], vert_cos[i - 1], vert_cos[i + 1], vn):
                        continue
            candidates.append((i, turn_angle))

    if not candidates:
        return []

    # Фильтрация: min_spacing вершин между соседними углами
    filtered = [candidates[0]]
    for i in range(1, len(candidates)):
        idx, angle = candidates[i]
        prev_idx, prev_angle = filtered[-1]
        if idx - prev_idx < min_spacing:
            if angle > prev_angle:
                filtered[-1] = (idx, angle)
            continue
        filtered.append((idx, angle))

    return [idx for idx, _ in filtered]


def _split_open_chain_at_corners(raw_chain, corner_indices):
    """Split one open raw chain at interior corner indices into multiple sub-chains.

    Each corner vertex becomes the end of one sub-chain and the start of the next.
    """
    vert_indices = raw_chain.get("vert_indices", [])
    vert_cos = raw_chain.get("vert_cos", [])
    edge_indices = raw_chain.get("edge_indices", [])
    side_face_indices = raw_chain.get("side_face_indices", [])
    neighbor = raw_chain.get("neighbor", NB_MESH_BORDER)

    # Границы сегментов: [0, corner1, corner2, ..., len-1]
    boundaries = [0] + sorted(corner_indices) + [len(vert_indices) - 1]

    parent_start_loop_index = raw_chain.get("start_loop_index", 0)

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

        sub_chains.append({
            "vert_indices": seg_verts,
            "vert_cos": seg_cos,
            "edge_indices": seg_edges,
            "side_face_indices": seg_sides,
            "neighbor": neighbor,
            "is_closed": False,
            "start_loop_index": parent_start_loop_index + start,
            "end_loop_index": parent_start_loop_index + end,
        })

    return sub_chains if sub_chains else [raw_chain]


def _split_border_chains_by_corners(raw_chains, basis_u, basis_v, bm=None):
    """Split all open MESH_BORDER chains at geometric corners.

    Безусловно применяется ко всем MESH_BORDER chains — не проверяем
    classification заранее. Corner detection + split, потом классификация
    каждого полученного сегмента по ratio.
    """
    result = []
    for raw_chain in raw_chains:
        neighbor = int(raw_chain.get("neighbor", NB_MESH_BORDER))
        is_closed = bool(raw_chain.get("is_closed", False))

        if neighbor != NB_MESH_BORDER or is_closed:
            result.append(raw_chain)
            continue

        vert_cos = raw_chain.get("vert_cos", [])
        if len(vert_cos) < 3:
            result.append(raw_chain)
            continue

        corner_indices = _find_open_chain_corners(
            vert_cos, basis_u, basis_v,
            vert_indices=raw_chain.get("vert_indices"),
            bm=bm,
        )
        if not corner_indices:
            result.append(raw_chain)
            continue

        sub_chains = _split_open_chain_at_corners(raw_chain, corner_indices)
        result.extend(sub_chains)

    return result


def _try_geometric_outer_loop_split(raw_loop, raw_chains, basis_u, basis_v, bm=None):
    """Fallback split for isolated OUTER loops that collapsed into one FREE chain."""

    loop_kind = raw_loop.get("kind", LoopKind.OUTER)
    if not isinstance(loop_kind, LoopKind):
        loop_kind = LoopKind(loop_kind)

    if loop_kind != LoopKind.OUTER or len(raw_chains) != 1:
        return raw_chains

    raw_chain = raw_chains[0]
    # Любой single closed chain на OUTER loop нужно пробовать split.
    # Ранее проверялся только FREE, но прямая стена (H_FRAME/V_FRAME)
    # тоже может быть single chain если все edges имеют одного соседа.
    if not raw_chain.get("is_closed", False):
        return raw_chains

    lvi = raw_loop.get("vert_indices")
    split_indices = _collect_geometric_split_indices(
        raw_loop.get("vert_cos", []), basis_u, basis_v,
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
        int(raw_chain.get("neighbor", NB_MESH_BORDER)),
    )
    print(f"[CFTUV][GeoSplit] derived_chains={len(derived_raw_chains)}")
    if len(derived_raw_chains) < 4:
        print(f"[CFTUV][GeoSplit] BAIL: <4 derived chains")
        return raw_chains

    derived_roles = [
        _classify_chain_frame_role(derived_raw_chain.get("vert_cos", []), basis_u, basis_v, strict_guards=False)
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
        chain_vert_cos = [co.copy() for co in raw_chain.get("vert_cos", [])]
        neighbor_id = int(raw_chain.get("neighbor", NB_MESH_BORDER))
        # MESH_BORDER chains: только ratio check (strict_guards=False)
        # PATCH/SEAM chains: ratio + ослабленные waviness/curvature guards
        use_strict_guards = (neighbor_id != NB_MESH_BORDER)
        chains.append(
            BoundaryChain(
                vert_indices=list(raw_chain.get("vert_indices", [])),
                vert_cos=chain_vert_cos,
                edge_indices=list(raw_chain.get("edge_indices", [])),
                side_face_indices=list(raw_chain.get("side_face_indices", [])),
                neighbor_patch_id=neighbor_id,
                is_closed=bool(raw_chain.get("is_closed", False)),
                frame_role=_classify_chain_frame_role(chain_vert_cos, basis_u, basis_v, strict_guards=use_strict_guards),
                start_loop_index=int(raw_chain.get("start_loop_index", 0)),
                end_loop_index=int(raw_chain.get("end_loop_index", 0)),
            )
        )
    return chains

def _build_loop_corners(boundary_loop, raw_chains, basis_u, basis_v):
    """Build loop corners from chain junctions or from geometric turns inside one loop."""

    chain_count = len(boundary_loop.chains)
    if chain_count < 2:
        return _build_geometric_loop_corners(boundary_loop, basis_u, basis_v)

    corners = []
    for next_chain_index in range(chain_count):
        prev_chain_index = (next_chain_index - 1) % chain_count
        prev_chain = boundary_loop.chains[prev_chain_index]
        next_chain = boundary_loop.chains[next_chain_index]
        raw_next_chain = raw_chains[next_chain_index]

        if next_chain.vert_cos:
            corner_co = next_chain.vert_cos[0].copy()
        elif prev_chain.vert_cos:
            corner_co = prev_chain.vert_cos[-1].copy()
        else:
            corner_co = Vector((0.0, 0.0, 0.0))

        prev_point = _find_corner_reference_point(prev_chain.vert_cos, corner_co, reverse=True)
        next_point = _find_corner_reference_point(next_chain.vert_cos, corner_co, reverse=False)
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)

        corners.append(
            BoundaryCorner(
                loop_vert_index=int(raw_next_chain.get("start_loop_index", 0)),
                vert_index=int(raw_next_chain.get("vert_indices", [-1])[0]) if raw_next_chain.get("vert_indices") else -1,
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

    for chain_index, chain in enumerate(boundary_loop.chains):
        chain.start_corner_index = chain_index
        chain.end_corner_index = (chain_index + 1) % chain_count


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

    for raw_loop in raw_loops:
        loop_kind = raw_loop.get("kind", LoopKind.OUTER)
        if not isinstance(loop_kind, LoopKind):
            loop_kind = LoopKind(loop_kind)

        boundary_loop = BoundaryLoop(
            vert_indices=list(raw_loop.get("vert_indices", [])),
            vert_cos=[co.copy() for co in raw_loop.get("vert_cos", [])],
            edge_indices=list(raw_loop.get("edge_indices", [])),
            side_face_indices=list(raw_loop.get("side_face_indices", [])),
            kind=loop_kind,
            depth=int(raw_loop.get("depth", 0)),
        )

        loop_neighbors = [
            _neighbor_for_side(edge_index, side_face_index, patch_face_indices, face_to_patch, patch_id, bm)
            for edge_index, side_face_index in zip(raw_loop.get("edge_indices", []), raw_loop.get("side_face_indices", []))
        ]
        raw_chains = _split_loop_into_chains_by_neighbor(
            raw_loop.get("vert_indices", []),
            raw_loop.get("vert_cos", []),
            raw_loop.get("edge_indices", []),
            raw_loop.get("side_face_indices", []),
            loop_neighbors,
        )
        # Мержим chains, разделённые бевель-заворотом: поворот вдоль нормали,
        # а не в касательной плоскости. На UV это один непрерывный сегмент.
        raw_chains = _merge_bevel_wrap_chains(raw_chains, bm)

        raw_chains = _try_geometric_outer_loop_split(raw_loop, raw_chains, basis_u, basis_v, bm=bm)
        raw_chains = _split_border_chains_by_corners(raw_chains, basis_u, basis_v, bm=bm)
        boundary_loop.chains = _build_boundary_chain_objects(raw_chains, basis_u, basis_v)

        boundary_loop.corners = _build_loop_corners(boundary_loop, raw_chains, basis_u, basis_v)
        _assign_loop_chain_endpoint_topology(boundary_loop)
        boundary_loops.append(boundary_loop)

    return boundary_loops


def _classify_loops_outer_hole(bm, raw_patch_data, obj=None):
    """Classify loops as OUTER or HOLE through a temporary UV unwrap."""

    if not raw_patch_data:
        return

    for patch_data in raw_patch_data.values():
        if len(patch_data["raw_loops"]) <= 1:
            for raw_loop in patch_data["raw_loops"]:
                raw_loop["kind"] = LoopKind.OUTER
                raw_loop["depth"] = 0

    if obj is None or obj.type != "MESH":
        return

    uv_temp_name = "_cftuv_temp"
    uv_layer = bm.loops.layers.uv.get(uv_temp_name)
    if uv_layer is None:
        uv_layer = bm.loops.layers.uv.new(uv_temp_name)

    original_active_uv_name = None
    original_selection = [face.index for face in bm.faces if face.select]

    try:
        bmesh.update_edit_mesh(obj.data)
        if obj.data.uv_layers.active:
            original_active_uv_name = obj.data.uv_layers.active.name
        if uv_temp_name in obj.data.uv_layers:
            obj.data.uv_layers[uv_temp_name].active = True

        for face in bm.faces:
            face.select = False

        for patch_data in raw_patch_data.values():
            raw_loops = patch_data["raw_loops"]
            if len(raw_loops) <= 1:
                continue

            patch_face_indices = list(patch_data["face_indices"])
            for face_idx in patch_face_indices:
                bm.faces[face_idx].select = True

            bmesh.update_edit_mesh(obj.data)
            bpy.ops.uv.unwrap(method="CONFORMAL", fill_holes=False, margin=0.0)

            for face_idx in patch_face_indices:
                bm.faces[face_idx].select = False

            _classify_raw_loops_via_uv(raw_loops, bm, patch_face_indices, uv_layer)
    finally:
        if uv_layer is not None:
            bm.loops.layers.uv.remove(uv_layer)

        for face in bm.faces:
            face.select = False
        for face_index in original_selection:
            if 0 <= face_index < len(bm.faces):
                bm.faces[face_index].select = True

        bmesh.update_edit_mesh(obj.data)
        if original_active_uv_name and original_active_uv_name in obj.data.uv_layers:
            obj.data.uv_layers[original_active_uv_name].active = True


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
        raw_patch_data[patch_id] = {
            "face_indices": list(patch_face_indices),
            "raw_loops": raw_loops,
        }

    _classify_loops_outer_hole(bm, raw_patch_data, obj)

    for patch_id, node in patch_graph.nodes.items():
        patch_data = raw_patch_data[patch_id]
        node.boundary_loops = _build_boundary_loops(
            patch_data["raw_loops"],
            patch_data["face_indices"],
            patch_graph.face_to_patch,
            patch_id,
            node.basis_u,
            node.basis_v,
            bm,
        )

        # FLOOR/SLOPE patches: H/V straightening искажает irregular caps.
        # Все chains → FREE, Conformal unwrap обработает целиком.
        if node.patch_type in (PatchType.FLOOR, PatchType.SLOPE):
            for bl in node.boundary_loops:
                for ch in bl.chains:
                    ch.frame_role = FrameRole.FREE

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


def format_patch_graph_report(graph, mesh_name=None):
    """Build text lines for the System Console PatchGraph report."""

    lines = []

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

    summary = (
        f"Patches: {total_patches} (W:{walls} F:{floors} S:{slopes} 1f:{singles}) | "
        f"Loops: {total_loops} Chains: {total_chains} Corners: {total_corners} Sharp:{total_sharp_corners} Holes: {total_holes} | "
        f"Roles: H:{total_h} V:{total_v} Free:{total_free} | "
        f"Facing: Up:{total_up} Down:{total_down} Side:{total_side} | "
        f"Neighbors: Patch:{total_patch_links} Self:{total_self_seams} Border:{total_mesh_borders}"
    )

    return lines, summary





