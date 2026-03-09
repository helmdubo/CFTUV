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
    from .model import BoundaryChain, BoundaryLoop, FrameRole, LoopKind, PatchGraph, PatchNode, PatchType, SeamEdge
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
    from model import BoundaryChain, BoundaryLoop, FrameRole, LoopKind, PatchGraph, PatchNode, PatchType, SeamEdge


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


def get_expanded_islands(bm, initial_faces):
    """ÃÂÃÂ°Ã‘â€¦ÃÂ¾ÃÂ´ÃÂ¸Ã‘â€š full/core islands ÃÂ´ÃÂ»Ã‘Â two-pass unwrap ÃÂ½ÃÂ° ÃÂ¾Ã‘ÂÃÂ½ÃÂ¾ÃÂ²ÃÂµ seam/sharp ÃÂ³Ã‘â‚¬ÃÂ°ÃÂ½ÃÂ¸Ã‘â€ ."""

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
                if edge.seam or not edge.smooth:
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
    """Flood fill patch-ÃÂµÃÂ¹, Ã‘â‚¬ÃÂ°ÃÂ·ÃÂ´ÃÂµÃÂ»Ã‘â€˜ÃÂ½ÃÂ½Ã‘â€¹Ã‘â€¦ seam/sharp edges."""

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
                if edge.seam or not edge.smooth:
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
    """Dual-strategy ÃÂ¿ÃÂ¾ÃÂ¸Ã‘ÂÃÂº ÃÂ»ÃÂ¾ÃÂºÃÂ°ÃÂ»Ã‘Å’ÃÂ½ÃÂ¾ÃÂ³ÃÂ¾ up-ÃÂ½ÃÂ°ÃÂ¿Ã‘â‚¬ÃÂ°ÃÂ²ÃÂ»ÃÂµÃÂ½ÃÂ¸Ã‘Â ÃÂ´ÃÂ»Ã‘Â patch."""

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
    """ÃÅ¡ÃÂ»ÃÂ°Ã‘ÂÃ‘ÂÃÂ¸Ã‘â€žÃÂ¸Ã‘â€ ÃÂ¸Ã‘â‚¬Ã‘Æ’ÃÂµÃ‘â€š patch ÃÂ¸ ÃÂ²Ã‘â€¹Ã‘â€¡ÃÂ¸Ã‘ÂÃÂ»Ã‘ÂÃÂµÃ‘â€š ÃÂ±ÃÂ°ÃÂ·ÃÂ¾ÃÂ²Ã‘â€¹ÃÂµ ÃÂ¼ÃÂµÃ‘â€šÃ‘â‚¬ÃÂ¸ÃÂºÃÂ¸."""

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

    up_dot = abs(avg_normal.dot(WORLD_UP))
    if up_dot > FLOOR_THRESHOLD:
        patch_type = PatchType.FLOOR
    elif up_dot < WALL_THRESHOLD:
        patch_type = PatchType.WALL
    else:
        patch_type = PatchType.SLOPE

    return patch_type, avg_normal, total_area, perimeter


def _calc_surface_basis(normal, ref_up=WORLD_UP):
    up_proj = ref_up - normal * ref_up.dot(normal)
    if up_proj.length_squared < 1e-5:
        tangent = Vector((1.0, 0.0, 0.0))
        tangent = (tangent - normal * tangent.dot(normal)).normalized()
        return tangent, normal.cross(tangent).normalized()
    bitangent = up_proj.normalized()
    return bitangent.cross(normal).normalized(), bitangent


def _build_patch_basis(bm, face_indices, patch_type, normal):
    """ÃÂ¡Ã‘â€šÃ‘â‚¬ÃÂ¾ÃÂ¸Ã‘â€š ÃÂ»ÃÂ¾ÃÂºÃÂ°ÃÂ»Ã‘Å’ÃÂ½Ã‘â€¹ÃÂ¹ basis_u / basis_v ÃÂ´ÃÂ»Ã‘Â patch."""

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
    """ÃÅ¸Ã‘â‚¬ÃÂ¾ÃÂ²ÃÂµÃ‘â‚¬Ã‘ÂÃÂµÃ‘â€š, Ã‘ÂÃÂ²ÃÂ»Ã‘ÂÃÂµÃ‘â€šÃ‘ÂÃ‘Â ÃÂ»ÃÂ¸ ÃÂºÃÂ¾ÃÂ½ÃÂºÃ‘â‚¬ÃÂµÃ‘â€šÃÂ½ÃÂ°Ã‘Â face-side Ã‘â€¡ÃÂ°Ã‘ÂÃ‘â€šÃ‘Å’Ã‘Å½ boundary patch-ÃÂ°."""

    edge = loop.edge
    in_patch_count = sum(1 for linked_face in edge.link_faces if linked_face.index in patch_face_indices)
    if len(edge.link_faces) == 1:
        return True
    if in_patch_count == 1:
        return True
    if in_patch_count >= 2 and (edge.seam or not edge.smooth):
        return True
    return False


def _boundary_side_key(loop):
    return (loop.face.index, loop.edge.index, loop.vert.index)


def _find_next_boundary_side(loop, patch_face_indices):
    """ÃËœÃÂ´Ã‘â€˜Ã‘â€š ÃÂ¿ÃÂ¾ patch boundary Ã‘â€¡ÃÂµÃ‘â‚¬ÃÂµÃÂ· face loops, ÃÂ½ÃÂµ Ã‘ÂÃ‘â€¦ÃÂ»ÃÂ¾ÃÂ¿Ã‘â€¹ÃÂ²ÃÂ°Ã‘Â SEAM_SELF."""

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
    """ÃÂ¡ÃÂ¾ÃÂ±ÃÂ¸Ã‘â‚¬ÃÂ°ÃÂµÃ‘â€š boundary loops ÃÂ¿ÃÂ¾ face-side topology ÃÂ¸ Ã‘ÂÃ‘â‚¬ÃÂ°ÃÂ·Ã‘Æ’ Ã‘ÂÃÂµÃ‘â‚¬ÃÂ¸ÃÂ°ÃÂ»ÃÂ¸ÃÂ·Ã‘Æ’ÃÂµÃ‘â€š ÃÂ¸Ã‘â€¦."""

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
    """ÃÅ¾ÃÂ¿Ã‘â‚¬ÃÂµÃÂ´ÃÂµÃÂ»Ã‘ÂÃÂµÃ‘â€š Ã‘â€šÃÂ¸ÃÂ¿ Ã‘ÂÃÂ¾Ã‘ÂÃÂµÃÂ´ÃÂ° ÃÂ´ÃÂ»Ã‘Â boundary side ÃÂ¿ÃÂ¾ Ã‘ÂÃ‘â€šÃÂ°ÃÂ±ÃÂ¸ÃÂ»Ã‘Å’ÃÂ½Ã‘â€¹ÃÂ¼ ÃÂ¸ÃÂ½ÃÂ´ÃÂµÃÂºÃ‘ÂÃÂ°ÃÂ¼."""

    edge = bm.edges[edge_index]
    if len(edge.link_faces) == 1:
        return NB_MESH_BORDER

    in_patch_faces = [linked_face for linked_face in edge.link_faces if linked_face.index in patch_face_indices]
    if len(in_patch_faces) >= 2 and (edge.seam or not edge.smooth):
        return NB_SEAM_SELF

    other_faces = [linked_face for linked_face in edge.link_faces if linked_face.index not in patch_face_indices]
    if not other_faces:
        return NB_MESH_BORDER

    neighbor_patch_id = face_to_patch.get(other_faces[0].index, NB_MESH_BORDER)
    if neighbor_patch_id == patch_id:
        return NB_SEAM_SELF

    return neighbor_patch_id


def _split_loop_into_chains_by_neighbor(loop_vert_cos, loop_edge_indices, loop_neighbors):
    """ÃÂ ÃÂ°ÃÂ·ÃÂ±ÃÂ¸ÃÂ²ÃÂ°ÃÂµÃ‘â€š boundary loop ÃÂ½ÃÂ° chains ÃÂ¿ÃÂ¾ Ã‘ÂÃÂ¼ÃÂµÃÂ½ÃÂµ Ã‘ÂÃÂ¾Ã‘ÂÃÂµÃÂ´ÃÂ½ÃÂµÃÂ³ÃÂ¾ patch/border/self-seam."""

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
                "vert_cos": list(loop_vert_cos),
                "edge_indices": list(loop_edge_indices),
                "neighbor": neighbors[0],
                "is_closed": True,
            }
        ]

    chains = []
    split_count = len(split_indices)
    for split_idx in range(split_count):
        v_start = split_indices[split_idx]
        v_end = split_indices[(split_idx + 1) % split_count]

        chain_vert_cos = []
        chain_edge_indices = []
        idx = v_start
        safety = 0
        while safety < vertex_count + 2:
            safety += 1
            chain_vert_cos.append(loop_vert_cos[idx % vertex_count])
            chain_edge_indices.append(loop_edge_indices[idx % edge_count])
            idx += 1
            if idx % vertex_count == v_end % vertex_count:
                chain_vert_cos.append(loop_vert_cos[v_end % vertex_count])
                break

        chains.append(
            {
                "vert_cos": chain_vert_cos,
                "edge_indices": chain_edge_indices,
                "neighbor": neighbors[v_start % edge_count],
                "is_closed": False,
            }
        )

    return chains


def _classify_raw_loops_via_uv(raw_loops, bm, patch_face_indices, uv_layer):
    """ÃÅ¡ÃÂ»ÃÂ°Ã‘ÂÃ‘ÂÃÂ¸Ã‘â€žÃÂ¸Ã‘â€ ÃÂ¸Ã‘â‚¬Ã‘Æ’ÃÂµÃ‘â€š loops ÃÂ½ÃÂ° OUTER/HOLE Ã‘â€¡ÃÂµÃ‘â‚¬ÃÂµÃÂ· nesting ÃÂ² 2D UV."""

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


def _find_loop_corners(loop_vert_cos, angle_threshold_deg=CORNER_ANGLE_THRESHOLD_DEG):
    """ÃÂÃÂ°Ã‘â€¦ÃÂ¾ÃÂ´ÃÂ¸Ã‘â€š corner vertices ÃÂ½ÃÂ° boundary loop."""

    count = len(loop_vert_cos)
    if count < 3:
        return []

    cos_threshold = math.cos(math.radians(angle_threshold_deg))
    corners = []

    for idx in range(count):
        vert_prev = loop_vert_cos[(idx - 1) % count]
        vert_curr = loop_vert_cos[idx]
        vert_next = loop_vert_cos[(idx + 1) % count]

        dir_prev = vert_curr - vert_prev
        dir_next = vert_next - vert_curr

        if dir_prev.length < 1e-8 or dir_next.length < 1e-8:
            corners.append(idx)
            continue

        cos_angle = dir_prev.normalized().dot(dir_next.normalized())
        if cos_angle < cos_threshold:
            corners.append(idx)

    return corners


def _split_loop_into_segments(loop_vert_cos, corners):
    """ÃÂ ÃÂ°ÃÂ·ÃÂ±ÃÂ¸ÃÂ²ÃÂ°ÃÂµÃ‘â€š ÃÂ·ÃÂ°ÃÂ¼ÃÂºÃÂ½Ã‘Æ’Ã‘â€šÃ‘â€¹ÃÂ¹ loop ÃÂ½ÃÂ° Ã‘ÂÃÂµÃÂ³ÃÂ¼ÃÂµÃÂ½Ã‘â€šÃ‘â€¹ ÃÂ¿ÃÂ¾ corner vertices."""

    count = len(loop_vert_cos)
    if not corners:
        return [list(loop_vert_cos)]

    segments = []
    corner_count = len(corners)
    for corner_index in range(corner_count):
        start_idx = corners[corner_index]
        end_idx = corners[(corner_index + 1) % corner_count]

        segment = []
        idx = start_idx
        while True:
            segment.append(loop_vert_cos[idx % count])
            if idx % count == end_idx % count:
                break
            idx += 1
            if len(segment) > count + 1:
                break

        if len(segment) >= 2:
            segments.append(segment)

    return segments


def _classify_segment_frame_role(segment_vert_cos, basis_u, basis_v, threshold=FRAME_ALIGNMENT_THRESHOLD):
    """ÃÅ¾ÃÂ¿Ã‘â‚¬ÃÂµÃÂ´ÃÂµÃÂ»Ã‘ÂÃÂµÃ‘â€š frame role Ã‘ÂÃÂµÃÂ³ÃÂ¼ÃÂµÃÂ½Ã‘â€šÃÂ° loop-ÃÂ°."""

    if len(segment_vert_cos) < 2:
        return FrameRole.FREE

    us = [vert_co.dot(basis_u) for vert_co in segment_vert_cos]
    vs = [vert_co.dot(basis_v) for vert_co in segment_vert_cos]

    extent_u = max(us) - min(us)
    extent_v = max(vs) - min(vs)
    total_extent = max(extent_u, extent_v)
    if total_extent < 1e-6:
        return FrameRole.FREE

    ratio_v = extent_v / total_extent
    ratio_u = extent_u / total_extent

    if ratio_v < threshold:
        return FrameRole.H_FRAME
    if ratio_u < threshold:
        return FrameRole.V_FRAME
    return FrameRole.FREE


def _compute_centroid(bm, face_indices):
    """Ãâ€™Ã‘â€¹Ã‘â€¡ÃÂ¸Ã‘ÂÃÂ»Ã‘ÂÃÂµÃ‘â€š centroid patch-ÃÂ° ÃÂºÃÂ°ÃÂº Ã‘ÂÃ‘â‚¬ÃÂµÃÂ´ÃÂ½ÃÂµÃÂµ ÃÂ¿ÃÂ¾ ÃÂ²ÃÂµÃ‘â‚¬Ã‘Ë†ÃÂ¸ÃÂ½ÃÂ°ÃÂ¼ faces."""

    center = Vector((0.0, 0.0, 0.0))
    count = 0
    for face_idx in face_indices:
        face = bm.faces[face_idx]
        for vert in face.verts:
            center += vert.co
            count += 1
    return center / max(count, 1)


def _serialize_patch_geometry(bm, face_indices):
    """ÃÂ¡ÃÂµÃ‘â‚¬ÃÂ¸ÃÂ°ÃÂ»ÃÂ¸ÃÂ·Ã‘Æ’ÃÂµÃ‘â€š ÃÂ³ÃÂµÃÂ¾ÃÂ¼ÃÂµÃ‘â€šÃ‘â‚¬ÃÂ¸Ã‘Å½ patch-ÃÂ° ÃÂ² verts/tris ÃÂ´ÃÂ»Ã‘Â debug ÃÂ¸ ÃÂ¾Ã‘â€šÃ‘â€¡Ã‘â€˜Ã‘â€šÃÂ¾ÃÂ²."""

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
    """ÃÂ¡ÃÂµÃ‘â‚¬ÃÂ¸ÃÂ°ÃÂ»ÃÂ¸ÃÂ·Ã‘Æ’ÃÂµÃ‘â€š raw loops ÃÂ² BoundaryLoop/BoundaryChain Ã‘ÂÃ‘â€šÃ‘â‚¬Ã‘Æ’ÃÂºÃ‘â€šÃ‘Æ’Ã‘â‚¬Ã‘â€¹."""

    boundary_loops = []
    patch_face_indices = set(patch_face_indices)

    for raw_loop in raw_loops:
        loop_kind = raw_loop.get("kind", LoopKind.OUTER)
        if not isinstance(loop_kind, LoopKind):
            loop_kind = LoopKind(loop_kind)

        boundary_loop = BoundaryLoop(
            vert_cos=[co.copy() for co in raw_loop.get("vert_cos", [])],
            edge_indices=list(raw_loop.get("edge_indices", [])),
            kind=loop_kind,
            depth=int(raw_loop.get("depth", 0)),
        )

        loop_neighbors = [
            _neighbor_for_side(edge_index, side_face_index, patch_face_indices, face_to_patch, patch_id, bm)
            for edge_index, side_face_index in zip(raw_loop.get("edge_indices", []), raw_loop.get("side_face_indices", []))
        ]
        raw_chains = _split_loop_into_chains_by_neighbor(
            raw_loop.get("vert_cos", []),
            raw_loop.get("edge_indices", []),
            loop_neighbors,
        )
        boundary_loop.chains = [
            BoundaryChain(
                vert_cos=[co.copy() for co in raw_chain.get("vert_cos", [])],
                edge_indices=list(raw_chain.get("edge_indices", [])),
                neighbor_patch_id=int(raw_chain.get("neighbor", NB_MESH_BORDER)),
                is_closed=bool(raw_chain.get("is_closed", False)),
                frame_role=FrameRole.FREE,
            )
            for raw_chain in raw_chains
        ]

        corners = _find_loop_corners(raw_loop.get("vert_cos", []))
        segments = _split_loop_into_segments(raw_loop.get("vert_cos", []), corners)
        boundary_loop.segments = []
        for segment_vert_cos in segments:
            frame_role = _classify_segment_frame_role(segment_vert_cos, basis_u, basis_v)
            boundary_loop.segments.append(
                {
                    "vert_cos": [co.copy() for co in segment_vert_cos],
                    "frame_role": frame_role,
                    "loop_kind": boundary_loop.kind,
                }
            )

        boundary_loops.append(boundary_loop)

    return boundary_loops


def _classify_loops_outer_hole(bm, raw_patch_data, obj=None):
    """ÃÅ¡ÃÂ»ÃÂ°Ã‘ÂÃ‘ÂÃÂ¸Ã‘â€žÃÂ¸Ã‘â€ ÃÂ¸Ã‘â‚¬Ã‘Æ’ÃÂµÃ‘â€š loops ÃÂ½ÃÂ° OUTER/HOLE Ã‘â€¡ÃÂµÃ‘â‚¬ÃÂµÃÂ· ÃÂ²Ã‘â‚¬ÃÂµÃÂ¼ÃÂµÃÂ½ÃÂ½Ã‘â€¹ÃÂ¹ unwrap.

    ÃÂ­Ã‘â€šÃÂ¾ ÃÂµÃÂ´ÃÂ¸ÃÂ½Ã‘ÂÃ‘â€šÃÂ²ÃÂµÃÂ½ÃÂ½ÃÂ°Ã‘Â helper-Ã‘â€žÃ‘Æ’ÃÂ½ÃÂºÃ‘â€ ÃÂ¸Ã‘Â analysis.py Ã‘Â side effects ÃÂ² UV/selection state.
    """

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
    original_seams = [edge.seam for edge in bm.edges]

    try:
        for edge in bm.edges:
            if not edge.smooth:
                edge.seam = True

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
            bpy.ops.uv.unwrap(method="CONFORMAL", margin=0.0)

            for face_idx in patch_face_indices:
                bm.faces[face_idx].select = False

            _classify_raw_loops_via_uv(raw_loops, bm, patch_face_indices, uv_layer)
    finally:
        for edge_index, edge in enumerate(bm.edges):
            edge.seam = original_seams[edge_index]

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
    """ÃÂ¡Ã‘â€šÃ‘â‚¬ÃÂ¾ÃÂ¸Ã‘â€š SeamEdge Ã‘ÂÃÂ²Ã‘ÂÃÂ·ÃÂ¸ ÃÂ¼ÃÂµÃÂ¶ÃÂ´Ã‘Æ’ Ã‘ÂÃÂ¾Ã‘ÂÃÂµÃÂ´ÃÂ½ÃÂ¸ÃÂ¼ÃÂ¸ patch-ÃÂ°ÃÂ¼ÃÂ¸."""

    seam_links = {}

    for edge in bm.edges:
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
    """ÃÂ¡Ã‘â€šÃ‘â‚¬ÃÂ¾ÃÂ¸Ã‘â€š ÃÂ¿ÃÂ¾ÃÂ»ÃÂ½Ã‘â€¹ÃÂ¹ PatchGraph ÃÂ¸ÃÂ· faces Ã‘â€šÃÂµÃÂºÃ‘Æ’Ã‘â€°ÃÂµÃÂ³ÃÂ¾ selection/ÃÂ¾ÃÂ¿ÃÂµÃ‘â‚¬ÃÂ°Ã‘â€ ÃÂ¸ÃÂ¸."""

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
        patch_type, normal, area, perimeter = _classify_patch(bm, patch_face_indices)
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

    for seam_edge in _build_seam_edges(patch_graph.face_to_patch, bm):
        patch_graph.add_edge(seam_edge)

    return patch_graph


def format_patch_graph_report(graph):
    """Ãâ€œÃÂ¾Ã‘â€šÃÂ¾ÃÂ²ÃÂ¸Ã‘â€š Ã‘ÂÃ‘â€šÃ‘â‚¬ÃÂ¾ÃÂºÃÂ¸ Ã‘â€šÃÂµÃÂºÃ‘ÂÃ‘â€šÃÂ¾ÃÂ²ÃÂ¾ÃÂ³ÃÂ¾ debug-ÃÂ¾Ã‘â€šÃ‘â€¡Ã‘â€˜Ã‘â€šÃÂ° ÃÂ´ÃÂ»Ã‘Â System Console."""

    lines = []

    total_patches = len(graph.nodes)
    walls = 0
    floors = 0
    slopes = 0
    singles = 0
    total_loops = 0
    total_chains = 0
    total_holes = 0
    total_segments = 0
    total_h = 0
    total_v = 0
    total_free = 0

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        patch_type = _enum_value(node.patch_type)
        if patch_type == PatchType.WALL.value:
            walls += 1
        elif patch_type == PatchType.FLOOR.value:
            floors += 1
        elif patch_type == PatchType.SLOPE.value:
            slopes += 1

        if len(node.face_indices) == 1:
            singles += 1

        chain_count = 0
        roles = []
        chain_neighbors = []
        loop_kinds = []
        loop_details = []

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            total_loops += 1
            loop_kind = _enum_value(boundary_loop.kind)
            loop_kinds.append(loop_kind)
            if loop_kind == LoopKind.HOLE.value:
                total_holes += 1

            chain_count += len(boundary_loop.chains)
            loop_chain_neighbors = []
            for chain in boundary_loop.chains:
                neighbor_label = f"nb={chain.neighbor_patch_id}"
                chain_neighbors.append(neighbor_label)
                loop_chain_neighbors.append(neighbor_label)

            loop_details.append(
                f"    Loop {loop_index}: {loop_kind} | chains:{len(boundary_loop.chains)} "
                f"[{' '.join(loop_chain_neighbors)}]"
            )

            for segment in boundary_loop.segments:
                role = _enum_value(segment.get("frame_role", FrameRole.FREE))
                roles.append(role)
                total_segments += 1
                if role == FrameRole.H_FRAME.value:
                    total_h += 1
                elif role == FrameRole.V_FRAME.value:
                    total_v += 1
                else:
                    total_free += 1

        total_chains += chain_count
        lines.append(
            f"  Patch {patch_id}: {patch_type} | {len(node.face_indices)}f | "
            f"loops:{len(node.boundary_loops)}[{' '.join(loop_kinds)}] chains:{chain_count} | "
            f"segs:[{' '.join(roles)}] | "
            f"chains:[{' '.join(chain_neighbors)}]"
        )
        lines.extend(loop_details)

    summary = (
        f"Patches: {total_patches} (W:{walls} F:{floors} S:{slopes} 1f:{singles}) | "
        f"Loops: {total_loops} Chains: {total_chains} Holes: {total_holes} | "
        f"Seg: {total_segments} H:{total_h} V:{total_v} Free:{total_free}"
    )

    return lines, summary
