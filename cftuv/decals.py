"""CFTUV Decal Producer (Phase 3) — генерация mesh-декалей из PatchGraph.

Потребитель PatchGraph (аналогично debug.py): исходный BMesh не читается,
вся топология и геометрия берётся из графа (vert_cos в локальном
пространстве source object). Модуль строит новую геометрию лент в
собственном bmesh и материализует её отдельными объектами в коллекции
Decals_Generated с матрицей источника.

Логика перенесена из прототипа hotspotingUV_mesh_decals_Full.py
(v1.2.0) и переписана на PatchGraph-цепочки:

- TRIM TOP/BOTTOM: граничные рёбра WALL patches классифицируются по
  направлению «наружу» в плоскости стены (edge_dir x normal против
  basis_v), собираются в ориентированные ribbon-runs с сохранением
  ChainRef/owner-frame и выдавливаются с адаптивной биссектрисой на стыках.
- CORNERS: WALL-WALL цепочки с непараллельными нормалями — спайн вдоль
  полилинии цепочки (работает на изогнутых углах) + два крыла по стенам.
- SEAMS: копланарные WALL-WALL цепочки — плоская лента по шву.

UV лент пишутся в прямоугольники атласа (DECAL_UV_RECT_*), продольная
координата — длина дуги * uv_length_scale (final_scale плотности).
"""

from dataclasses import dataclass

import bpy
import bmesh
from mathutils import Vector

from .constants import (
    DECAL_COLLECTION_NAME,
    DECAL_COPLANAR_DOT,
    DECAL_DIR_THRESHOLD,
    DECAL_NOISE_THRESHOLD,
    DECAL_SPINE_MERGE_DISTANCE,
    DECAL_UV_RECT_BOTTOM,
    DECAL_UV_RECT_CORNER,
    DECAL_UV_RECT_SEAM,
    DECAL_UV_RECT_TOP,
    DECAL_WELD_DISTANCE,
    WORLD_UP,
)
from .model import ChainNeighborKind, ChainRef, DecalSettings, PatchGraph, PatchType

DECAL_MODES = ("TOP", "BOTTOM", "CORNERS", "SEAMS")

_MODE_OBJECT_SUFFIX = {
    "TOP": "Top",
    "BOTTOM": "Bottom",
    "CORNERS": "Corners",
    "SEAMS": "Seams",
}


@dataclass
class _OrientedRibbonRun:
    """Ориентированный путь ленты с owner-frame на каждый сегмент.

    В отличие от голого списка рёбер run не теряет источник и нормаль
    поверхности при склейке chains. Список segment_* всегда на один элемент
    короче vert_indices/points.
    """

    vert_indices: list[int]
    points: list[Vector]
    segment_normals: list[Vector]
    segment_ups: list[Vector]
    segment_chain_refs: list[ChainRef]

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
        return _OrientedRibbonRun(
            vert_indices=list(reversed(self.vert_indices)),
            points=list(reversed(self.points)),
            segment_normals=list(reversed(self.segment_normals)),
            segment_ups=list(reversed(self.segment_ups)),
            segment_chain_refs=list(reversed(self.segment_chain_refs)),
        )


def _join_ribbon_runs(first, second):
    """Соединяет два уже ориентированных run с общей конечной вершиной."""

    if first.end_vert_index != second.start_vert_index:
        raise ValueError("Ribbon runs do not share an oriented endpoint")
    return _OrientedRibbonRun(
        vert_indices=first.vert_indices + second.vert_indices[1:],
        points=first.points + second.points[1:],
        segment_normals=first.segment_normals + second.segment_normals,
        segment_ups=first.segment_ups + second.segment_ups,
        segment_chain_refs=first.segment_chain_refs + second.segment_chain_refs,
    )


# ============================================================
# Сбор кандидатов из PatchGraph (чистая геометрия, без bpy)
# ============================================================


def _edge_key(vert_a: int, vert_b: int) -> tuple[int, int]:
    return (vert_a, vert_b) if vert_a < vert_b else (vert_b, vert_a)


def _chain_segment_surface_frame(node, chain, segment_index):
    """Local owner normal/up for one boundary segment, with legacy fallback."""

    normal = node.normal.copy()
    if segment_index < len(chain.side_face_normals):
        local_normal = chain.side_face_normals[segment_index]
        if local_normal.length_squared > 1e-12:
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


def _chain_ref_is_enabled(chain_refs, chain_ref: ChainRef) -> bool:
    return chain_refs is None or chain_ref in chain_refs


def _chain_eligible_for_trim(graph: PatchGraph, chain) -> bool:
    """Кромка WALL patch, не являющаяся швом WALL-WALL.

    WALL-WALL швы обслуживаются corner/seam декалями — исключаем,
    чтобы не было двойных лент на одном ребре. SEAM_SELF — внутренний
    шов патча, физической кромки нет.
    """

    kind = chain.neighbor_kind
    if kind == ChainNeighborKind.MESH_BORDER:
        return True
    if kind == ChainNeighborKind.PATCH:
        neighbor = graph.nodes.get(chain.neighbor_patch_id)
        return neighbor is not None and neighbor.patch_type != PatchType.WALL
    return False


def _collect_trim_ribbon_runs(graph: PatchGraph, chain_refs=None):
    """Собирает TOP/BOTTOM как ориентированные chain-derived segment runs.

    Каждый сегмент сохраняет owner normal/up и ChainRef. Дальнейшая склейка
    может развернуть порядок точек для непрерывного обхода, но не теряет
    желаемую нормаль поверхности и потому способна исправить winding quad.
    """

    top_runs = []
    bottom_runs = []
    seen_edge_keys = set()
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        if node.patch_type != PatchType.WALL:
            continue
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                chain_ref = (patch_id, loop_index, chain_index)
                if not _chain_ref_is_enabled(chain_refs, chain_ref):
                    continue
                if not _chain_eligible_for_trim(graph, chain):
                    continue
                verts = chain.vert_indices
                points = chain.vert_cos
                if len(verts) < 2 or len(verts) != len(points):
                    continue
                edge_count = len(verts) if chain.is_closed else len(verts) - 1
                for index in range(edge_count):
                    next_index = (index + 1) % len(verts)
                    normal, up = _chain_segment_surface_frame(node, chain, index)
                    segment = points[next_index] - points[index]
                    if segment.length < DECAL_NOISE_THRESHOLD:
                        continue
                    outward = segment.normalized().cross(normal)
                    if outward.length_squared < 1e-8:
                        continue
                    score = outward.normalized().dot(up)
                    if score > DECAL_DIR_THRESHOLD:
                        bucket = top_runs
                    elif score < -DECAL_DIR_THRESHOLD:
                        bucket = bottom_runs
                    else:
                        continue
                    edge_key = _edge_key(verts[index], verts[next_index])
                    if edge_key in seen_edge_keys:
                        continue
                    seen_edge_keys.add(edge_key)
                    bucket.append(
                        _OrientedRibbonRun(
                            vert_indices=[verts[index], verts[next_index]],
                            points=[points[index].copy(), points[next_index].copy()],
                            segment_normals=[normal.copy()],
                            segment_ups=[up.copy()],
                            segment_chain_refs=[chain_ref],
                        )
                    )
    return _stitch_ribbon_runs(top_runs), _stitch_ribbon_runs(bottom_runs)


def _stitch_ribbon_runs(runs):
    """Склеивает run по концам, но не проходит через неоднозначный point-contact.

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
            current = _join_ribbon_runs(current, next_run)

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
            current = _join_ribbon_runs(previous_run, current)

        stitched.append(current)
    return stitched


def _blend_ribbon_frame(frame_prev, frame_next):
    if frame_prev and frame_next:
        normal = frame_prev[0] + frame_next[0]
        up = frame_prev[1] + frame_next[1]
        if normal.length_squared < 1e-8 or up.length_squared < 1e-8:
            return frame_prev
        return normal.normalized(), up.normalized()
    if frame_prev:
        return frame_prev
    if frame_next:
        return frame_next
    return Vector((0.0, 0.0, 1.0)), Vector((0.0, 1.0, 0.0))


def _ribbon_vertex_frames(run: _OrientedRibbonRun):
    """Фреймы вершин без неориентированного edge-key lookup."""

    segment_count = len(run.segment_normals)
    if segment_count == 0:
        return []
    is_ring = run.is_closed
    frames = []
    for point_index in range(len(run.points)):
        previous_index = point_index - 1
        next_index = point_index
        if point_index == 0:
            previous_index = segment_count - 1 if is_ring else -1
        if point_index >= segment_count:
            next_index = 0 if is_ring else -1
        frame_prev = None
        frame_next = None
        if 0 <= previous_index < segment_count:
            frame_prev = (
                run.segment_normals[previous_index],
                run.segment_ups[previous_index],
            )
        if 0 <= next_index < segment_count:
            frame_next = (
                run.segment_normals[next_index],
                run.segment_ups[next_index],
            )
        frames.append(_blend_ribbon_frame(frame_prev, frame_next))
    return frames


def _trim_quad_requires_flip(
    point_a, point_b, extrude_a, extrude_b, desired_normal
):
    """True, если стандартный quad winding смотрит против owner surface."""

    tangent = point_b - point_a
    transverse = extrude_a + extrude_b
    if transverse.length_squared < 1e-12:
        transverse = extrude_a
    geometric_normal = tangent.cross(transverse)
    if (
        geometric_normal.length_squared < 1e-12
        or desired_normal.length_squared < 1e-12
    ):
        return False
    return geometric_normal.dot(desired_normal) < 0.0


def _trim_quad_layout(
    base_a,
    base_b,
    tip_a,
    tip_b,
    u_start,
    u_end,
    v_min,
    v_max,
    flip_winding,
):
    """Возвращает winding и UV, сохраняя base=v_min, tip=v_max."""

    if flip_winding:
        return (
            (base_a, tip_a, tip_b, base_b),
            (
                (u_start, v_min),
                (u_start, v_max),
                (u_end, v_max),
                (u_end, v_min),
            ),
        )
    return (
        (base_a, base_b, tip_b, tip_a),
        (
            (u_start, v_min),
            (u_end, v_min),
            (u_end, v_max),
            (u_start, v_max),
        ),
    )


def _chain_edge_signature(chain):
    return tuple(sorted(int(index) for index in chain.edge_indices))


def _chain_representative_surface_normal(node, chain):
    normal_sum = Vector((0.0, 0.0, 0.0))
    for normal in chain.side_face_normals:
        if normal.length_squared > 1e-12:
            normal_sum += normal.normalized()
    if normal_sum.length_squared > 1e-12:
        return normal_sum.normalized()
    return node.normal.copy()


def _collect_paired_chain_items(graph, chain_refs=None, wall_only=False):
    """PATCH и парные SEAM_SELF chain uses как двухсторонние поверхности."""

    uses_by_owner_and_edges = {}
    all_uses = []
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                chain_ref = (patch_id, loop_index, chain_index)
                signature = _chain_edge_signature(chain)
                use = (chain_ref, node, chain)
                all_uses.append(use)
                if signature:
                    uses_by_owner_and_edges.setdefault(
                        (patch_id, signature), []
                    ).append(use)

    items = []
    pair_kinds = []
    consumed_refs = set()
    seen_pairs = set()
    for chain_ref, node, chain in all_uses:
        patch_id = chain_ref[0]
        if wall_only and node.patch_type != PatchType.WALL:
            continue
        if not _chain_ref_is_enabled(chain_refs, chain_ref):
            continue
        if len(chain.vert_cos) < 2:
            continue
        signature = _chain_edge_signature(chain)
        if not signature:
            continue

        counterpart = None
        if chain.neighbor_kind == ChainNeighborKind.PATCH:
            neighbor = graph.nodes.get(chain.neighbor_patch_id)
            if neighbor is None or (wall_only and neighbor.patch_type != PatchType.WALL):
                continue
            pair_key = (
                "PATCH",
                min(patch_id, chain.neighbor_patch_id),
                max(patch_id, chain.neighbor_patch_id),
                signature,
            )
            candidates = uses_by_owner_and_edges.get(
                (chain.neighbor_patch_id, signature), ()
            )
            counterpart = candidates[0] if candidates else None
            counterpart_node = neighbor
        elif chain.neighbor_kind == ChainNeighborKind.SEAM_SELF:
            pair_key = ("SEAM_SELF", patch_id, signature)
            candidates = uses_by_owner_and_edges.get((patch_id, signature), ())
            if len(candidates) != 2:
                continue
            counterpart = next(
                (candidate for candidate in candidates if candidate[0] != chain_ref),
                None,
            )
            counterpart_node = node
        else:
            continue

        if pair_key in seen_pairs:
            continue
        seen_pairs.add(pair_key)
        normal_a = _chain_representative_surface_normal(node, chain)
        if counterpart is not None:
            other_ref, other_node, other_chain = counterpart
            normal_b = _chain_representative_surface_normal(other_node, other_chain)
            consumed_refs.add(other_ref)
        else:
            normal_b = counterpart_node.normal.copy()
        consumed_refs.add(chain_ref)
        items.append(
            (
                list(chain.vert_cos),
                normal_a,
                normal_b,
                chain.is_closed,
                chain.dihedral_convexity,
            )
        )
        pair_kinds.append(chain.neighbor_kind)
    return items, consumed_refs, pair_kinds


def _collect_wall_pair_chains(graph: PatchGraph, chain_refs=None):
    """WALL surface pairs, включая две стороны одного SEAM_SELF."""

    corner_chains = []
    seam_chains = []
    paired_items, _consumed_refs, pair_kinds = _collect_paired_chain_items(
        graph, chain_refs=chain_refs, wall_only=True
    )
    for item, pair_kind in zip(paired_items, pair_kinds):
        _points, normal_a, normal_b, _is_closed, _convexity = item
        if (
            pair_kind == ChainNeighborKind.PATCH
            and normal_a.dot(normal_b) > DECAL_COPLANAR_DOT
        ):
            seam_chains.append(item)
        else:
            corner_chains.append(item)
    return corner_chains, seam_chains


def _collect_manual_chain_decals(graph: PatchGraph, chain_refs):
    """Выбранные chains без semantic-фильтра patch type.

    PATCH-neighbor и парный SEAM_SELF дают угловую декаль по двум локальным
    поверхностям. Непарный MESH_BORDER/SEAM_SELF даёт одно крыло на owner patch.
    Две стороны seam дедуплицируются по source mesh edge indices.
    """

    corner_chains, consumed_refs, _pair_kinds = _collect_paired_chain_items(
        graph, chain_refs=chain_refs, wall_only=False
    )
    boundary_chains = []
    seen_boundary_chains = set()

    for patch_id, loop_index, chain_index in sorted(chain_refs or ()):
        node = graph.nodes.get(patch_id)
        if node is None or not (0 <= loop_index < len(node.boundary_loops)):
            continue
        boundary_loop = node.boundary_loops[loop_index]
        if not (0 <= chain_index < len(boundary_loop.chains)):
            continue
        chain = boundary_loop.chains[chain_index]
        if len(chain.vert_cos) < 2:
            continue
        chain_ref = (patch_id, loop_index, chain_index)
        if chain_ref in consumed_refs:
            continue

        edge_signature = _chain_edge_signature(chain)
        boundary_key = edge_signature or (patch_id, loop_index, chain_index)
        if boundary_key in seen_boundary_chains:
            continue
        seen_boundary_chains.add(boundary_key)
        boundary_chains.append(
            (
                list(chain.vert_cos),
                _chain_representative_surface_normal(node, chain),
                chain.is_closed,
            )
        )

    return corner_chains, boundary_chains


def _manual_edge_pair_convexity(points, normal_a, normal_b):
    if len(points) < 2:
        return 0.0
    direction = points[1] - points[0]
    if direction.length_squared < 1e-12:
        return 0.0
    cross = direction.normalized().cross(normal_a)
    if cross.length_squared < 1e-12:
        return 0.0
    value = cross.normalized().dot(normal_b)
    if abs(value) < 0.01:
        return 0.0
    return max(-1.0, min(1.0, value))


def _collect_manual_edge_decals(graph: PatchGraph, edge_indices):
    """Selected physical edges paired from their analysis-owned chain uses."""

    selected_edges = {int(edge_index) for edge_index in edge_indices or ()}
    uses_by_edge = {edge_index: [] for edge_index in selected_edges}
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                for segment_index, edge_index in enumerate(chain.edge_indices):
                    edge_index = int(edge_index)
                    if edge_index not in selected_edges or len(chain.vert_cos) < 2:
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
                                chain.vert_cos[segment_index].copy(),
                                chain.vert_cos[next_index].copy(),
                            ],
                            normal,
                        )
                    )

    paired_edges = []
    boundary_edges = []
    for edge_index in sorted(selected_edges):
        uses = sorted(uses_by_edge.get(edge_index, ()), key=lambda use: use[0])
        if len(uses) >= 2:
            _ref_a, points, normal_a = uses[0]
            _ref_b, _other_points, normal_b = uses[1]
            paired_edges.append(
                (
                    points,
                    normal_a,
                    normal_b,
                    False,
                    _manual_edge_pair_convexity(points, normal_a, normal_b),
                )
            )
        elif len(uses) == 1:
            _ref, points, normal = uses[0]
            boundary_edges.append((points, normal, False))
    return paired_edges, boundary_edges


def _polyline_tangents(points, closed=False):
    """Касательная на вершину полилинии — среднее направлений соседних сегментов.

    closed=True для замкнутой полилинии в форме points[0] == points[-1]
    (дублированная точка замыкания): оба конца получают одинаковую
    биссектрису последнего и первого сегментов.
    """

    tangents = []
    count = len(points)
    wrap = closed and count > 2
    for i in range(count):
        tangent = Vector((0.0, 0.0, 0.0))
        if i > 0:
            delta = points[i] - points[i - 1]
        elif wrap:
            delta = points[-1] - points[-2]
        else:
            delta = None
        if delta is not None and delta.length_squared > 1e-12:
            tangent = tangent + delta.normalized()
        if i < count - 1:
            delta = points[i + 1] - points[i]
        elif wrap:
            delta = points[1] - points[0]
        else:
            delta = None
        if delta is not None and delta.length_squared > 1e-12:
            tangent = tangent + delta.normalized()
        if tangent.length_squared > 1e-12:
            tangents.append(tangent.normalized())
        else:
            tangents.append(Vector((0.0, 0.0, 1.0)))
    return tangents


def _dedupe_polyline(points, min_dist=DECAL_SPINE_MERGE_DISTANCE):
    """Схлопывает последовательные точки ближе min_dist (как в прототипе)."""

    if not points:
        return []
    result = [points[0]]
    for point in points[1:]:
        if (point - result[-1]).length > min_dist:
            result.append(point)
    return result


def _arc_lengths(points):
    """Кумулятивная длина дуги по вершинам полилинии (от 0.0)."""

    lengths = [0.0]
    for i in range(1, len(points)):
        lengths.append(lengths[-1] + (points[i] - points[i - 1]).length)
    return lengths


# ============================================================
# Построение геометрии лент (bmesh декали, не исходный BMesh)
# ============================================================


def _build_trim_strip(bm, run, settings, is_top, uv_rect):
    """Лента трима вдоль пути кромки.

    База — точка кромки + normal * offset, вытяжка вниз (TOP) или вверх
    (BOTTOM) вдоль адаптивного up на height_trim. UV: u — длина дуги,
    v — поперёк прямоугольника атласа.
    """

    if len(run.points) < 2:
        return
    points = run.points
    frames = _ribbon_vertex_frames(run)
    if len(frames) != len(points):
        return
    arc = _arc_lengths(points)

    base_verts = []
    tip_verts = []
    extrude_vectors = []
    for i, co in enumerate(points):
        normal, up = frames[i]
        extrude_dir = -up if is_top else up
        base_pos = co + normal * settings.offset
        base_verts.append(bm.verts.new(base_pos))
        tip_verts.append(bm.verts.new(base_pos + extrude_dir * settings.height_trim))
        extrude_vectors.append(extrude_dir)

    uv_layer = bm.loops.layers.uv.verify()
    u_min, v_min, u_max, v_max = uv_rect
    scale = settings.uv_length_scale
    for i in range(len(points) - 1):
        u_start = arc[i] * scale
        u_end = arc[i + 1] * scale
        flip_winding = _trim_quad_requires_flip(
            points[i],
            points[i + 1],
            extrude_vectors[i],
            extrude_vectors[i + 1],
            run.segment_normals[i],
        )
        quad_verts, quad_uvs = _trim_quad_layout(
            base_verts[i],
            base_verts[i + 1],
            tip_verts[i],
            tip_verts[i + 1],
            u_start,
            u_end,
            v_min,
            v_max,
            flip_winding,
        )
        try:
            face = bm.faces.new(quad_verts)
        except ValueError:
            continue
        for loop, uv in zip(face.loops, quad_uvs):
            loop[uv_layer].uv = uv


def _corner_wing_directions(direction, normal_a, normal_b, dihedral_convexity=0.0):
    """Направления крыльев вдоль обеих поверхностей с учётом inner/outer."""

    wing_dir_a = direction.cross(normal_a)
    wing_dir_b = direction.cross(normal_b)
    if wing_dir_a.length_squared < 1e-8 or wing_dir_b.length_squared < 1e-8:
        return None

    wing_dir_a = wing_dir_a.normalized()
    wing_dir_b = wing_dir_b.normalized()
    is_concave = dihedral_convexity < -0.01
    if (wing_dir_a.dot(normal_b) < 0.0) == is_concave:
        wing_dir_a = wing_dir_a * -1.0
    if (wing_dir_b.dot(normal_a) < 0.0) == is_concave:
        wing_dir_b = wing_dir_b * -1.0
    return wing_dir_a, wing_dir_b


def _build_corner_strip(
    bm,
    points,
    normal_a,
    normal_b,
    settings,
    uv_rect,
    closed=False,
    dihedral_convexity=0.0,
    width=None,
):
    """Угловая лента: спайн вдоль полилинии шва + крыло на каждую стену.

    Спайн смещается вдоль средней нормали на offset / dot — постоянное
    расстояние до обеих плоскостей. Направления крыльев считаются на
    вершину из касательной (изогнутые углы, Г-образные швы). UV: v —
    длина дуги, u — поперёк, спайн на середине прямоугольника.
    """

    if len(points) < 2:
        return
    avg_normal = normal_a + normal_b
    if avg_normal.length_squared < 1e-8:
        avg_normal = normal_a.copy()
    avg_normal = avg_normal.normalized()
    dot_val = avg_normal.dot(normal_a)
    scaler = settings.offset if abs(dot_val) < 0.1 else settings.offset / dot_val
    spine_offset = avg_normal * scaler
    half_width = (
        settings.width_corner if width is None else float(width)
    ) / 2.0

    tangents = _polyline_tangents(points, closed=closed)
    arc = _arc_lengths(points)

    spine_verts = []
    wing_a_verts = []
    wing_b_verts = []
    for i, co in enumerate(points):
        wing_directions = _corner_wing_directions(
            tangents[i], normal_a, normal_b, dihedral_convexity
        )
        if wing_directions is None:
            chord = points[-1] - points[0]
            wing_directions = _corner_wing_directions(
                chord, normal_a, normal_b, dihedral_convexity
            )
            if wing_directions is None:
                return
        wing_dir_a, wing_dir_b = wing_directions

        spine_pos = co + spine_offset
        spine_verts.append(bm.verts.new(spine_pos))
        wing_a_verts.append(bm.verts.new(spine_pos + wing_dir_a * half_width))
        wing_b_verts.append(bm.verts.new(spine_pos + wing_dir_b * half_width))

    uv_layer = bm.loops.layers.uv.verify()
    u_min, v_min, u_max, v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    scale = settings.uv_length_scale
    for i in range(len(points) - 1):
        v_start = arc[i] * scale
        v_end = arc[i + 1] * scale
        try:
            face_a = bm.faces.new(
                (wing_a_verts[i], wing_a_verts[i + 1], spine_verts[i + 1], spine_verts[i])
            )
            face_a.loops[0][uv_layer].uv = (u_min, v_start)
            face_a.loops[1][uv_layer].uv = (u_min, v_end)
            face_a.loops[2][uv_layer].uv = (u_mid, v_end)
            face_a.loops[3][uv_layer].uv = (u_mid, v_start)
            face_b = bm.faces.new(
                (spine_verts[i], spine_verts[i + 1], wing_b_verts[i + 1], wing_b_verts[i])
            )
            face_b.loops[0][uv_layer].uv = (u_mid, v_start)
            face_b.loops[1][uv_layer].uv = (u_mid, v_end)
            face_b.loops[2][uv_layer].uv = (u_max, v_end)
            face_b.loops[3][uv_layer].uv = (u_max, v_start)
        except ValueError:
            continue


def _boundary_wing_direction(direction, normal):
    """Направление единственного крыла от boundary внутрь owner patch."""

    wing_dir = normal.cross(direction)
    if wing_dir.length_squared < 1e-8:
        return None
    return wing_dir.normalized()


def _build_boundary_wing_strip(
    bm, points, normal, settings, uv_rect, closed=False, width=None
):
    """Одно corner-крыло от boundary chain внутрь owner patch."""

    if len(points) < 2:
        return
    tangents = _polyline_tangents(points, closed=closed)
    chord = points[-1] - points[0]
    arc = _arc_lengths(points)
    wing_width = (
        settings.width_corner if width is None else float(width)
    ) / 2.0

    spine_verts = []
    wing_verts = []
    for index, co in enumerate(points):
        wing_dir = _boundary_wing_direction(tangents[index], normal)
        if wing_dir is None:
            wing_dir = _boundary_wing_direction(chord, normal)
        if wing_dir is None:
            return
        spine_pos = co + normal * settings.offset
        spine_verts.append(bm.verts.new(spine_pos))
        wing_verts.append(bm.verts.new(spine_pos + wing_dir * wing_width))

    uv_layer = bm.loops.layers.uv.verify()
    u_min, _v_min, u_max, _v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    scale = settings.uv_length_scale
    for index in range(len(points) - 1):
        try:
            face = bm.faces.new(
                (
                    spine_verts[index],
                    spine_verts[index + 1],
                    wing_verts[index + 1],
                    wing_verts[index],
                )
            )
        except ValueError:
            continue
        v_start = arc[index] * scale
        v_end = arc[index + 1] * scale
        face.loops[0][uv_layer].uv = (u_mid, v_start)
        face.loops[1][uv_layer].uv = (u_mid, v_end)
        face.loops[2][uv_layer].uv = (u_max, v_end)
        face.loops[3][uv_layer].uv = (u_max, v_start)


def _build_seam_strip(bm, points, normal, settings, uv_rect, closed=False):
    """Плоская лента по копланарному шву, центрированная на полилинии."""

    if len(points) < 2:
        return
    offset_vec = normal * settings.offset
    half_width = settings.width_seam / 2.0
    tangents = _polyline_tangents(points, closed=closed)
    arc = _arc_lengths(points)

    left_verts = []
    right_verts = []
    for i, co in enumerate(points):
        wing_dir = tangents[i].cross(normal)
        if wing_dir.length_squared > 1e-8:
            wing_dir = wing_dir.normalized()
        else:
            wing_dir = Vector((1.0, 0.0, 0.0))
        pos = co + offset_vec
        left_verts.append(bm.verts.new(pos - wing_dir * half_width))
        right_verts.append(bm.verts.new(pos + wing_dir * half_width))

    uv_layer = bm.loops.layers.uv.verify()
    u_min, v_min, u_max, v_max = uv_rect
    scale = settings.uv_length_scale
    for i in range(len(points) - 1):
        try:
            face = bm.faces.new(
                (left_verts[i], left_verts[i + 1], right_verts[i + 1], right_verts[i])
            )
        except ValueError:
            continue
        v_start = arc[i] * scale
        v_end = arc[i + 1] * scale
        face.loops[0][uv_layer].uv = (u_min, v_start)
        face.loops[1][uv_layer].uv = (u_min, v_end)
        face.loops[2][uv_layer].uv = (u_max, v_end)
        face.loops[3][uv_layer].uv = (u_max, v_start)


# ============================================================
# Материализация объектов
# ============================================================


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


def _finalize_decal_object(bm, name: str, source_obj, scene):
    """Сваривает ленты, заменяет одноимённый объект прошлой генерации."""

    try:
        bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=DECAL_WELD_DISTANCE)
        loose_verts = [vert for vert in bm.verts if not vert.link_faces]
        if loose_verts:
            bmesh.ops.delete(bm, geom=loose_verts, context="VERTS")
        if not bm.faces:
            bm.free()
            return None

        old_obj = bpy.data.objects.get(name)
        if old_obj is not None:
            if old_obj.mode == "EDIT":
                # Старая декаль в edit-сессии — удалять нельзя (живой
                # BMEditMesh); новая получит суффикс имени от Blender.
                print(f"[CFTUV][Decals] '{name}' is in Edit Mode — keeping it, new object will be suffixed")
            else:
                old_mesh = old_obj.data
                bpy.data.objects.remove(old_obj, do_unlink=True)
                if old_mesh is not None and old_mesh.users == 0:
                    bpy.data.meshes.remove(old_mesh)

        mesh = bpy.data.meshes.new(name + "_Geo")
        bm.to_mesh(mesh)
    except Exception:
        bm.free()
        raise
    bm.free()

    obj = bpy.data.objects.new(name, mesh)
    _decal_collection(scene).objects.link(obj)
    obj.matrix_world = source_obj.matrix_world.copy()
    return obj


def _closed_polyline(points, is_closed):
    """Замыкает полилинию закрытой цепочки; шов сварится в финализации."""

    if is_closed and len(points) > 2:
        return points + [points[0]]
    return points


def _fill_manual_chain_decals(
    bm,
    graph,
    settings,
    chain_refs,
    mode="CORNERS",
    selected_edge_indices=None,
):
    """Manual edge scope: exact physical edges with local owner-side frames."""

    if selected_edge_indices is None:
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )
    else:
        corner_chains, boundary_chains = _collect_manual_edge_decals(
            graph, selected_edge_indices
        )
    is_seam_mode = mode == "SEAMS"
    width = settings.width_seam if is_seam_mode else settings.width_corner
    uv_rect = DECAL_UV_RECT_SEAM if is_seam_mode else DECAL_UV_RECT_CORNER
    for points, normal_a, normal_b, is_closed, convexity in corner_chains:
        spine_points = _dedupe_polyline(points)
        if is_seam_mode and normal_a.dot(normal_b) > DECAL_COPLANAR_DOT:
            _build_seam_strip(
                bm,
                _closed_polyline(spine_points, is_closed),
                normal_a,
                settings,
                uv_rect,
                closed=is_closed,
            )
            continue
        _build_corner_strip(
            bm,
            _closed_polyline(spine_points, is_closed),
            normal_a,
            normal_b,
            settings,
            uv_rect,
            closed=is_closed,
            dihedral_convexity=convexity,
            width=width,
        )
    for points, normal, is_closed in boundary_chains:
        spine_points = _dedupe_polyline(points)
        _build_boundary_wing_strip(
            bm,
            _closed_polyline(spine_points, is_closed),
            normal,
            settings,
            uv_rect,
            closed=is_closed,
            width=width,
        )


def _fill_decal_bmesh(
    bm,
    graph: PatchGraph,
    settings: DecalSettings,
    mode: str,
    chain_refs=None,
    selected_edge_indices=None,
):
    if chain_refs is not None and mode in ("CORNERS", "SEAMS"):
        _fill_manual_chain_decals(
            bm,
            graph,
            settings,
            chain_refs,
            mode=mode,
            selected_edge_indices=selected_edge_indices,
        )
    elif mode in ("TOP", "BOTTOM"):
        top_runs, bottom_runs = _collect_trim_ribbon_runs(
            graph, chain_refs=chain_refs
        )
        runs = top_runs if mode == "TOP" else bottom_runs
        uv_rect = DECAL_UV_RECT_TOP if mode == "TOP" else DECAL_UV_RECT_BOTTOM
        for run in runs:
            _build_trim_strip(bm, run, settings, mode == "TOP", uv_rect)
    elif mode == "CORNERS":
        corner_chains, _seam_chains = _collect_wall_pair_chains(graph)
        for points, normal_a, normal_b, is_closed, convexity in corner_chains:
            spine_points = _dedupe_polyline(points)
            _build_corner_strip(
                bm,
                _closed_polyline(spine_points, is_closed),
                normal_a,
                normal_b,
                settings,
                DECAL_UV_RECT_CORNER,
                closed=is_closed,
                dihedral_convexity=convexity,
            )
    elif mode == "SEAMS":
        _corner_chains, automatic_seam_chains = _collect_wall_pair_chains(graph)
        seam_chains = [
            (points, normal_a, is_closed)
            for points, normal_a, _normal_b, is_closed, _convexity
            in automatic_seam_chains
        ]
        for points, normal, is_closed in seam_chains:
            # Нормаль стороны-владельца (как n_sum_a в прототипе);
            # automatic seam pair копланарна в пределах порога.
            _build_seam_strip(
                bm,
                _closed_polyline(points, is_closed),
                normal,
                settings,
                DECAL_UV_RECT_SEAM,
                closed=is_closed,
            )


def generate_decal_objects(
    graph: PatchGraph,
    source_obj,
    settings: DecalSettings,
    mode: str,
    scene=None,
    chain_refs=None,
    selected_edge_indices=None,
) -> list[str]:
    """Генерирует decal-объект выбранного режима. Возвращает имена созданных."""

    if mode not in DECAL_MODES:
        raise ValueError(f"Unknown decal mode: {mode}")
    if scene is None:
        scene = bpy.context.scene

    bm = bmesh.new()
    try:
        _fill_decal_bmesh(
            bm,
            graph,
            settings,
            mode,
            chain_refs=chain_refs,
            selected_edge_indices=selected_edge_indices,
        )
    except Exception:
        bm.free()
        raise

    obj = _finalize_decal_object(
        bm, _decal_object_name(mode, source_obj), source_obj, scene
    )
    return [obj.name] if obj is not None else []
