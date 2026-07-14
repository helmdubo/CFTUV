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
from math import atan2, pi

import bpy
import bmesh
from mathutils import Vector

from .constants import (
    DECAL_COLLECTION_NAME,
    DECAL_COPLANAR_DOT,
    DECAL_CORNER_JOIN_GAP_RATIO,
    DECAL_CORNER_MITER_LIMIT,
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
from .decal_network import _corner_wing_directions, build_seam_network_faces
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
        # от направления обхода, ни от порядка сторон (см. flip-логику
        # `_corner_wing_directions`). Негация при reverse/swap выворачивала
        # крылья наружу после stitch-разворота.
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
class _CornerWingJoin:
    """Входящая/выходящая точки offset-контура на одной станции крыла."""

    incoming: Vector
    outgoing: Vector
    is_bevel: bool


@dataclass(frozen=True)
class _DecalSurfaceSection:
    """Поперечное ребро одной branch на конкретной offset-поверхности."""

    normal: Vector
    spine_vert: object
    verts: tuple
    uvs: tuple


@dataclass(frozen=True)
class _DecalJunctionPort:
    """Endpoint ribbon-run, обрезанный по границе junction patch."""

    source_vert_index: int
    outward_tangent: Vector
    sections: tuple[_DecalSurfaceSection, ...]


@dataclass(frozen=True)
class _DecalJunctionSpec:
    """Derived decal-only T/Y junction; не является solve entity."""

    source_vert_index: int
    source_pos: Vector
    core_pos: Vector


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


def _stitch_ribbon_runs(runs):
    return _stitch_oriented_runs(runs, _join_ribbon_runs)


def _stitch_corner_runs(runs):
    return _stitch_oriented_runs(runs, _join_corner_runs)


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
    """Selected physical edges as continuous analysis-owned corner runs."""

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
                        )
                    )

    paired_segments = []
    boundary_uses = {}
    for edge_index in sorted(selected_edges):
        uses = sorted(uses_by_edge.get(edge_index, ()), key=lambda use: use[0])
        if len(uses) >= 2:
            _ref_a, vert_indices, points, normal_a = uses[0]
            _ref_b, _other_verts, _other_points, normal_b = uses[1]
            paired_segments.append(
                _OrientedCornerRun(
                    vert_indices=vert_indices,
                    points=points,
                    segment_normals_a=[normal_a],
                    segment_normals_b=[normal_b],
                    segment_convexities=[
                        _manual_edge_pair_convexity(
                            points, normal_a, normal_b
                        )
                    ],
                    segment_edge_indices=[edge_index],
                )
            )
        elif len(uses) == 1:
            ref, vert_indices, points, normal = uses[0]
            boundary_uses.setdefault(ref[:3], []).append(
                (ref[3], edge_index, vert_indices, points, normal)
            )
    return (
        _stitch_corner_runs(paired_segments),
        _group_boundary_runs(boundary_uses),
    )


def _group_boundary_runs(boundary_uses):
    """Односторонние boundary-runs по последовательным сегментам одной chain.

    Wing задаётся chain boundary-ориентацией (материал слева, n × t),
    поэтому runs строятся строго в порядке обхода chain и не реверсируются
    generic-ститчером. Сторона A пустая (нулевые нормали) — признак
    односторонней ветви для decal_network.
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


# `_corner_wing_directions` перенесена в decal_network.py (общая для
# legacy miter pipeline и network backend) и реэкспортируется отсюда.


def _corner_station_segment_indices(run, point_index):
    """Соседние segment indices для станции, включая wrap замкнутого run."""

    segment_count = len(run.segment_normals_a)
    previous_index = point_index - 1
    next_index = point_index
    if point_index == 0:
        previous_index = segment_count - 1 if run.is_closed else -1
    if point_index >= segment_count:
        next_index = 0 if run.is_closed else -1
    return previous_index, next_index


def _corner_offset_join(
    spine_pos,
    previous_tangent,
    previous_wing_dir,
    next_tangent,
    next_wing_dir,
    wing_width,
    miter_limit=DECAL_CORNER_MITER_LIMIT,
):
    """Строит constant-width MITER или две точки BEVEL для одного крыла.

    MITER — пересечение offset-линий соседних сегментов. Если линии skew,
    параллельны с разными offset или пересечение дальше miter_limit, станция
    получает две независимые точки, которые builder закрывает bevel-гранью.
    """

    if previous_tangent is None or previous_wing_dir is None:
        point = spine_pos + next_wing_dir * wing_width
        return _CornerWingJoin(point, point.copy(), False)
    if next_tangent is None or next_wing_dir is None:
        point = spine_pos + previous_wing_dir * wing_width
        return _CornerWingJoin(point, point.copy(), False)

    incoming = spine_pos + previous_wing_dir * wing_width
    outgoing = spine_pos + next_wing_dir * wing_width
    prev_dir = previous_tangent.normalized()
    next_dir = next_tangent.normalized()
    direction_dot = prev_dir.dot(next_dir)
    denominator = 1.0 - direction_dot * direction_dot
    gap_limit = max(1e-6, wing_width * DECAL_CORNER_JOIN_GAP_RATIO)

    if denominator < 1e-8:
        if direction_dot > 0.0 and (incoming - outgoing).length <= gap_limit:
            point = (incoming + outgoing) * 0.5
            return _CornerWingJoin(point, point.copy(), False)
        return _CornerWingJoin(incoming, outgoing, True)

    between = incoming - outgoing
    prev_parameter = (
        direction_dot * next_dir.dot(between) - prev_dir.dot(between)
    ) / denominator
    next_parameter = (
        next_dir.dot(between) - direction_dot * prev_dir.dot(between)
    ) / denominator
    closest_previous = incoming + prev_dir * prev_parameter
    closest_next = outgoing + next_dir * next_parameter
    if (closest_previous - closest_next).length > gap_limit:
        return _CornerWingJoin(incoming, outgoing, True)

    miter = (closest_previous + closest_next) * 0.5
    if (miter - spine_pos).length > wing_width * miter_limit:
        return _CornerWingJoin(incoming, outgoing, True)
    return _CornerWingJoin(miter, miter.copy(), False)


def _corner_segment_wing_frames(run):
    """Segment-local tangent и направления обоих крыльев."""

    frames = []
    for index in range(len(run.points) - 1):
        tangent = run.points[index + 1] - run.points[index]
        if tangent.length_squared < 1e-12:
            return None
        tangent.normalize()
        wings = _corner_wing_directions(
            tangent,
            run.segment_normals_a[index],
            run.segment_normals_b[index],
            run.segment_convexities[index],
        )
        if wings is None:
            return None
        frames.append((tangent, wings[0], wings[1]))
    return frames


def _corner_run_wing_joins(run, spine_positions, wing_width):
    """Join-пары A/B для всех станций непрерывного corner run."""

    segment_frames = _corner_segment_wing_frames(run)
    if segment_frames is None:
        return None
    joins_a = []
    joins_b = []
    for point_index, spine_pos in enumerate(spine_positions):
        previous_index, next_index = _corner_station_segment_indices(
            run, point_index
        )
        previous = (
            segment_frames[previous_index]
            if 0 <= previous_index < len(segment_frames)
            else (None, None, None)
        )
        following = (
            segment_frames[next_index]
            if 0 <= next_index < len(segment_frames)
            else (None, None, None)
        )
        joins_a.append(
            _corner_offset_join(
                spine_pos,
                previous[0],
                previous[1],
                following[0],
                following[1],
                wing_width,
            )
        )
        joins_b.append(
            _corner_offset_join(
                spine_pos,
                previous[0],
                previous[2],
                following[0],
                following[2],
                wing_width,
            )
        )
    return joins_a, joins_b


def _blend_corner_normal(previous, following):
    if previous is not None and following is not None:
        blended = previous + following
        if blended.length_squared > 1e-8:
            return blended.normalized()
        return previous.copy()
    if previous is not None:
        return previous.copy()
    if following is not None:
        return following.copy()
    return Vector((0.0, 0.0, 1.0))


def _corner_run_vertex_frames(run: _OrientedCornerRun):
    """Смешивает две surface-нормали соседних сегментов на общей станции."""

    segment_count = len(run.segment_normals_a)
    if segment_count == 0:
        return []
    frames = []
    for point_index in range(len(run.points)):
        previous_index, next_index = _corner_station_segment_indices(
            run, point_index
        )

        prev_a = prev_b = next_a = next_b = None
        convexities = []
        if 0 <= previous_index < segment_count:
            prev_a = run.segment_normals_a[previous_index]
            prev_b = run.segment_normals_b[previous_index]
            convexities.append(run.segment_convexities[previous_index])
        if 0 <= next_index < segment_count:
            next_a = run.segment_normals_a[next_index]
            next_b = run.segment_normals_b[next_index]
            convexities.append(run.segment_convexities[next_index])
        convexity = sum(convexities) / len(convexities) if convexities else 0.0
        frames.append(
            (
                _blend_corner_normal(prev_a, next_a),
                _blend_corner_normal(prev_b, next_b),
                convexity,
            )
        )
    return frames


def _corner_spine_position(co, normal_a, normal_b, offset):
    """Точка spine на одинаковом расстоянии от двух owner surfaces."""

    avg_normal = normal_a + normal_b
    if avg_normal.length_squared < 1e-8:
        avg_normal = normal_a.copy()
    avg_normal.normalize()
    dot_val = avg_normal.dot(normal_a)
    scaler = offset if abs(dot_val) < 0.1 else offset / dot_val
    return co + avg_normal * scaler


def _unique_surface_normals(normals):
    """Группирует одинаковые oriented surface planes для decal junction."""

    unique = []
    for normal in normals:
        if normal.length_squared < 1e-12:
            continue
        candidate = normal.normalized()
        if any(candidate.dot(existing) > DECAL_COPLANAR_DOT for existing in unique):
            continue
        unique.append(candidate)
    return unique


def _offset_plane_junction_center(source_pos, normals, offset):
    """Least-squares intersection owner planes, сдвинутых на decal offset."""

    unique = _unique_surface_normals(normals)
    if not unique:
        return source_pos.copy()
    delta = Vector((0.0, 0.0, 0.0))
    for _iteration in range(64):
        correction = Vector((0.0, 0.0, 0.0))
        for normal in unique:
            correction += normal * (offset - normal.dot(delta))
        correction /= len(unique)
        delta += correction
        if correction.length_squared < 1e-18:
            break
    return source_pos + delta


def _junction_miter_position(
    core_pos,
    current_outer,
    current_tangent,
    next_outer,
    next_tangent,
    wing_width,
):
    """Пересекает outer-контуры branches для усреднённой divider-линии."""

    fallback = (current_outer + next_outer) * 0.5
    current_dir = current_tangent.normalized()
    next_dir = next_tangent.normalized()
    direction_dot = current_dir.dot(next_dir)
    denominator = 1.0 - direction_dot * direction_dot
    if denominator < 1e-8:
        return fallback

    between = current_outer - next_outer
    current_parameter = (
        direction_dot * next_dir.dot(between) - current_dir.dot(between)
    ) / denominator
    next_parameter = (
        next_dir.dot(between) - direction_dot * current_dir.dot(between)
    ) / denominator
    closest_current = current_outer + current_dir * current_parameter
    closest_next = next_outer + next_dir * next_parameter
    gap_limit = max(1e-6, wing_width * DECAL_CORNER_JOIN_GAP_RATIO)
    if (closest_current - closest_next).length > gap_limit:
        return fallback

    miter = (closest_current + closest_next) * 0.5
    if (miter - core_pos).length > wing_width * DECAL_CORNER_MITER_LIMIT:
        return fallback
    return miter


def _run_endpoint_data(run, at_start):
    segment_index = 0 if at_start else len(run.points) - 2
    point_index = 0 if at_start else len(run.points) - 1
    tangent = run.points[segment_index + 1] - run.points[segment_index]
    if not at_start:
        tangent *= -1.0
    if tangent.length_squared > 1e-12:
        tangent.normalize()
    return (
        run.points[point_index],
        tangent,
        run.segment_normals_a[segment_index],
        run.segment_normals_b[segment_index],
        (run.points[segment_index + 1] - run.points[segment_index]).length,
    )


def _prepare_seam_junctions(runs, settings, wing_width):
    """Находит valence 3+ endpoints и рассчитывает безопасный branch cut."""

    endpoint_map = {}
    for run_index, run in enumerate(runs):
        if run.is_closed:
            continue
        endpoint_map.setdefault(run.start_vert_index, []).append((run_index, True))
        endpoint_map.setdefault(run.end_vert_index, []).append((run_index, False))

    specs = {}
    cuts = {}
    for vert_index, endpoints in endpoint_map.items():
        if len(endpoints) < 3:
            continue
        first_run_index, first_at_start = endpoints[0]
        source_pos = _run_endpoint_data(
            runs[first_run_index], first_at_start
        )[0].copy()
        normals = []
        endpoint_data = []
        for run_index, at_start in endpoints:
            data = _run_endpoint_data(runs[run_index], at_start)
            endpoint_data.append((run_index, at_start, data))
            normals.extend((data[2], data[3]))
        core_pos = _offset_plane_junction_center(
            source_pos, normals, settings.offset
        )
        specs[vert_index] = _DecalJunctionSpec(
            source_vert_index=vert_index,
            source_pos=source_pos,
            core_pos=core_pos,
        )

        for run_index, at_start, data in endpoint_data:
            point, tangent, normal_a, normal_b, segment_length = data
            if normal_a.dot(normal_b) > DECAL_COPLANAR_DOT:
                normal = normal_a + normal_b
                if normal.length_squared < 1e-8:
                    normal = normal_a.copy()
                normal.normalize()
                endpoint_spine = point + normal * settings.offset
            else:
                endpoint_spine = _corner_spine_position(
                    point, normal_a, normal_b, settings.offset
                )
            desired_cut = wing_width + (core_pos - endpoint_spine).dot(tangent)
            max_cut = segment_length * 0.45
            min_cut = min(wing_width * 0.25, max_cut)
            cuts[(run_index, at_start)] = max(
                min_cut, min(desired_cut, max_cut)
            )
    return specs, cuts


def _trim_run_for_junctions(run, start_cut=0.0, end_cut=0.0):
    """Обрезает только geometry points, сохраняя chain/edge identity run."""

    points = [point.copy() for point in run.points]
    if start_cut > 0.0:
        tangent = points[1] - points[0]
        if tangent.length_squared > 1e-12:
            points[0] += tangent.normalized() * start_cut
    if end_cut > 0.0:
        tangent = points[-2] - points[-1]
        if tangent.length_squared > 1e-12:
            points[-1] += tangent.normalized() * end_cut
    return _OrientedCornerRun(
        vert_indices=list(run.vert_indices),
        points=points,
        segment_normals_a=list(run.segment_normals_a),
        segment_normals_b=list(run.segment_normals_b),
        segment_convexities=list(run.segment_convexities),
        segment_edge_indices=list(run.segment_edge_indices),
    )


def _endpoint_port(run, sections_start, sections_end):
    """Возвращает два ports открытого run после построения ribbon mesh."""

    if run.is_closed:
        return []
    start_tangent = run.points[1] - run.points[0]
    end_tangent = run.points[-2] - run.points[-1]
    if start_tangent.length_squared > 1e-12:
        start_tangent.normalize()
    if end_tangent.length_squared > 1e-12:
        end_tangent.normalize()
    return [
        _DecalJunctionPort(
            source_vert_index=run.start_vert_index,
            outward_tangent=start_tangent,
            sections=tuple(sections_start),
        ),
        _DecalJunctionPort(
            source_vert_index=run.end_vert_index,
            outward_tangent=end_tangent,
            sections=tuple(sections_end),
        ),
    ]


def _build_corner_ribbon_run(bm, run, settings, uv_rect, width=None):
    """Строит corner-ленту с constant-width joins на станциях run."""

    if len(run.points) < 2:
        return
    half_width = (
        settings.width_corner if width is None else float(width)
    ) / 2.0
    frames = _corner_run_vertex_frames(run)
    arc = _arc_lengths(run.points)

    spine_positions = []
    for index, co in enumerate(run.points):
        normal_a, normal_b, _convexity = frames[index]
        spine_positions.append(
            _corner_spine_position(
                co, normal_a, normal_b, settings.offset
            )
        )

    wing_joins = _corner_run_wing_joins(run, spine_positions, half_width)
    if wing_joins is None:
        return
    joins_a, joins_b = wing_joins
    spine_verts = []
    wing_a_incoming = []
    wing_a_outgoing = []
    wing_b_incoming = []
    wing_b_outgoing = []
    for spine_pos, join_a, join_b in zip(spine_positions, joins_a, joins_b):
        spine_verts.append(bm.verts.new(spine_pos))

        incoming_a = bm.verts.new(join_a.incoming)
        outgoing_a = (
            bm.verts.new(join_a.outgoing) if join_a.is_bevel else incoming_a
        )
        wing_a_incoming.append(incoming_a)
        wing_a_outgoing.append(outgoing_a)

        incoming_b = bm.verts.new(join_b.incoming)
        outgoing_b = (
            bm.verts.new(join_b.outgoing) if join_b.is_bevel else incoming_b
        )
        wing_b_incoming.append(incoming_b)
        wing_b_outgoing.append(outgoing_b)

    uv_layer = bm.loops.layers.uv.verify()
    u_min, _v_min, u_max, _v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    scale = settings.uv_length_scale

    def add_face(vertices, uvs):
        try:
            face = bm.faces.new(vertices)
        except ValueError:
            return
        for loop, uv in zip(face.loops, uvs):
            loop[uv_layer].uv = uv

    for index in range(len(run.points) - 1):
        v_start = arc[index] * scale
        v_end = arc[index + 1] * scale
        add_face(
            (
                wing_a_outgoing[index],
                wing_a_incoming[index + 1],
                spine_verts[index + 1],
                spine_verts[index],
            ),
            (
                (u_min, v_start),
                (u_min, v_end),
                (u_mid, v_end),
                (u_mid, v_start),
            ),
        )
        add_face(
            (
                spine_verts[index],
                spine_verts[index + 1],
                wing_b_incoming[index + 1],
                wing_b_outgoing[index],
            ),
            (
                (u_mid, v_start),
                (u_mid, v_end),
                (u_max, v_end),
                (u_max, v_start),
            ),
        )

    join_count = len(run.points) - 1 if run.is_closed else len(run.points)
    for index in range(join_count):
        if not run.is_closed and index in (0, len(run.points) - 1):
            continue
        v_station = arc[index] * scale
        if joins_a[index].is_bevel:
            v_span = max(
                (joins_a[index].incoming - joins_a[index].outgoing).length
                * scale
                * 0.5,
                1e-6,
            )
            add_face(
                (
                    spine_verts[index],
                    wing_a_incoming[index],
                    wing_a_outgoing[index],
                ),
                (
                    (u_mid, v_station),
                    (u_min, v_station - v_span),
                    (u_min, v_station + v_span),
                ),
            )
        if joins_b[index].is_bevel:
            v_span = max(
                (joins_b[index].incoming - joins_b[index].outgoing).length
                * scale
                * 0.5,
                1e-6,
            )
            add_face(
                (
                    spine_verts[index],
                    wing_b_outgoing[index],
                    wing_b_incoming[index],
                ),
                (
                    (u_mid, v_station),
                    (u_max, v_station + v_span),
                    (u_max, v_station - v_span),
                ),
            )

    first_segment = 0
    last_segment = len(run.segment_normals_a) - 1
    v_last = arc[-1] * scale
    return _endpoint_port(
        run,
        (
            _DecalSurfaceSection(
                run.segment_normals_a[first_segment],
                spine_verts[0],
                (spine_verts[0], wing_a_outgoing[0]),
                ((u_mid, 0.0), (u_min, 0.0)),
            ),
            _DecalSurfaceSection(
                run.segment_normals_b[first_segment],
                spine_verts[0],
                (spine_verts[0], wing_b_outgoing[0]),
                ((u_mid, 0.0), (u_max, 0.0)),
            ),
        ),
        (
            _DecalSurfaceSection(
                run.segment_normals_a[last_segment],
                spine_verts[-1],
                (spine_verts[-1], wing_a_incoming[-1]),
                ((u_mid, v_last), (u_min, v_last)),
            ),
            _DecalSurfaceSection(
                run.segment_normals_b[last_segment],
                spine_verts[-1],
                (spine_verts[-1], wing_b_incoming[-1]),
                ((u_mid, v_last), (u_max, v_last)),
            ),
        ),
    )


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
    """Compatibility wrapper для chain-level corner ribbon."""

    if len(points) < 2:
        return
    segment_count = len(points) - 1
    vert_indices = list(range(len(points)))
    if closed:
        vert_indices[-1] = vert_indices[0]
    run = _OrientedCornerRun(
        vert_indices=vert_indices,
        points=list(points),
        segment_normals_a=[normal_a.copy() for _ in range(segment_count)],
        segment_normals_b=[normal_b.copy() for _ in range(segment_count)],
        segment_convexities=[dihedral_convexity] * segment_count,
        segment_edge_indices=list(range(segment_count)),
    )
    _build_corner_ribbon_run(bm, run, settings, uv_rect, width=width)


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


def _build_selected_seam_ribbon_run(bm, run, settings, uv_rect):
    """Centered SEAMS ribbon по selected-edge run с MITER/BEVEL joins."""

    if len(run.points) < 2:
        return []
    half_width = settings.width_seam / 2.0
    segment_frames = []
    for index in range(len(run.points) - 1):
        tangent = run.points[index + 1] - run.points[index]
        if tangent.length_squared < 1e-12:
            return []
        tangent.normalize()
        normal = run.segment_normals_a[index] + run.segment_normals_b[index]
        if normal.length_squared < 1e-8:
            normal = run.segment_normals_a[index].copy()
        normal.normalize()
        wing = tangent.cross(normal)
        if wing.length_squared < 1e-8:
            return []
        wing.normalize()
        segment_frames.append((tangent, normal, -wing, wing))

    spine_positions = []
    joins_left = []
    joins_right = []
    for point_index, co in enumerate(run.points):
        previous_index, next_index = _corner_station_segment_indices(
            run, point_index
        )
        previous = (
            segment_frames[previous_index]
            if 0 <= previous_index < len(segment_frames)
            else (None, None, None, None)
        )
        following = (
            segment_frames[next_index]
            if 0 <= next_index < len(segment_frames)
            else (None, None, None, None)
        )
        normal = _blend_corner_normal(previous[1], following[1])
        spine_pos = co + normal * settings.offset
        spine_positions.append(spine_pos)
        joins_left.append(
            _corner_offset_join(
                spine_pos,
                previous[0],
                previous[2],
                following[0],
                following[2],
                half_width,
            )
        )
        joins_right.append(
            _corner_offset_join(
                spine_pos,
                previous[0],
                previous[3],
                following[0],
                following[3],
                half_width,
            )
        )

    spine_verts = []
    left_incoming = []
    left_outgoing = []
    right_incoming = []
    right_outgoing = []
    for spine_pos, left_join, right_join in zip(
        spine_positions, joins_left, joins_right
    ):
        spine_verts.append(bm.verts.new(spine_pos))
        left_in = bm.verts.new(left_join.incoming)
        left_out = bm.verts.new(left_join.outgoing) if left_join.is_bevel else left_in
        left_incoming.append(left_in)
        left_outgoing.append(left_out)
        right_in = bm.verts.new(right_join.incoming)
        right_out = (
            bm.verts.new(right_join.outgoing) if right_join.is_bevel else right_in
        )
        right_incoming.append(right_in)
        right_outgoing.append(right_out)

    uv_layer = bm.loops.layers.uv.verify()
    u_min, _v_min, u_max, _v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    scale = settings.uv_length_scale
    arc = _arc_lengths(run.points)

    def add_face(vertices, uvs):
        try:
            face = bm.faces.new(vertices)
        except ValueError:
            return
        for loop, uv in zip(face.loops, uvs):
            loop[uv_layer].uv = uv

    for index in range(len(run.points) - 1):
        v_start = arc[index] * scale
        v_end = arc[index + 1] * scale
        add_face(
            (
                left_outgoing[index],
                left_incoming[index + 1],
                spine_verts[index + 1],
                spine_verts[index],
            ),
            (
                (u_min, v_start),
                (u_min, v_end),
                (u_mid, v_end),
                (u_mid, v_start),
            ),
        )
        add_face(
            (
                spine_verts[index],
                spine_verts[index + 1],
                right_incoming[index + 1],
                right_outgoing[index],
            ),
            (
                (u_mid, v_start),
                (u_mid, v_end),
                (u_max, v_end),
                (u_max, v_start),
            ),
        )

    join_count = len(run.points) - 1 if run.is_closed else len(run.points)
    for index in range(join_count):
        if not run.is_closed and index in (0, len(run.points) - 1):
            continue
        v_station = arc[index] * scale
        if joins_left[index].is_bevel:
            add_face(
                (
                    spine_verts[index],
                    left_incoming[index],
                    left_outgoing[index],
                ),
                (
                    (u_mid, v_station),
                    (u_min, v_station),
                    (u_min, v_station + 1e-6),
                ),
            )
        if joins_right[index].is_bevel:
            add_face(
                (
                    spine_verts[index],
                    right_outgoing[index],
                    right_incoming[index],
                ),
                (
                    (u_mid, v_station),
                    (u_max, v_station + 1e-6),
                    (u_max, v_station),
                ),
            )

    first_normal = segment_frames[0][1]
    last_normal = segment_frames[-1][1]
    v_last = arc[-1] * scale
    return _endpoint_port(
        run,
        (
            _DecalSurfaceSection(
                first_normal,
                spine_verts[0],
                (left_outgoing[0], spine_verts[0], right_outgoing[0]),
                ((u_min, 0.0), (u_mid, 0.0), (u_max, 0.0)),
            ),
        ),
        (
            _DecalSurfaceSection(
                last_normal,
                spine_verts[-1],
                (left_incoming[-1], spine_verts[-1], right_incoming[-1]),
                ((u_min, v_last), (u_mid, v_last), (u_max, v_last)),
            ),
        ),
    )


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


def _materialize_network_faces(bm, network_faces, settings, uv_rect):
    """Материализует faces decal-сети в bmesh с shared вершинами по ключам.

    Ключи ('sv', vert) сшивают spine-станции между крыльями/поверхностями и
    junction cores; прочие вершины дедуплицируются по квантованной позиции
    внутри поверхности, остаточные совпадения сваривает финальный weld.
    """

    uv_layer = bm.loops.layers.uv.verify()
    u_min, _v_min, u_max, _v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    u_radius = (u_max - u_min) / 2.0
    scale = settings.uv_length_scale
    verts_by_key = {}
    created = 0

    for network_face in network_faces:
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
            if loop_data and loop_data[-1][0] is vert:
                continue
            loop_data.append((vert, u_frac, v_length))
        if len(loop_data) > 2 and loop_data[0][0] is loop_data[-1][0]:
            loop_data.pop()
        if len(loop_data) < 3:
            continue
        try:
            face = bm.faces.new(tuple(item[0] for item in loop_data))
        except ValueError:
            continue
        for loop, (_vert, u_frac, v_length) in zip(face.loops, loop_data):
            loop[uv_layer].uv = (
                u_mid + u_frac * u_radius,
                v_length * scale,
            )
        created += 1
    return created


def _build_seam_junctions(bm, specs, ports, settings, uv_rect):
    """Делит junction sectors усреднёнными miter-линиями от общего core."""

    ports_by_vertex = {}
    for port in ports:
        if port.source_vert_index in specs:
            ports_by_vertex.setdefault(port.source_vert_index, []).append(port)

    uv_layer = bm.loops.layers.uv.verify()
    u_min, _v_min, u_max, _v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    u_radius = (u_max - u_min) / 2.0
    wing_width = settings.width_seam / 2.0
    created = 0

    for vert_index, incident_ports in ports_by_vertex.items():
        if len(incident_ports) < 3:
            continue
        spec = specs[vert_index]
        core_vert = bm.verts.new(spec.core_pos)
        surface_groups = []
        for port in incident_ports:
            for section in port.sections:
                normal = section.normal.normalized()
                group = next(
                    (
                        candidate
                        for candidate in surface_groups
                        if normal.dot(candidate[0]) > DECAL_COPLANAR_DOT
                    ),
                    None,
                )
                if group is None:
                    group = [normal.copy(), []]
                    surface_groups.append(group)
                else:
                    blended = group[0] + normal
                    if blended.length_squared > 1e-8:
                        group[0] = blended.normalized()
                group[1].append((port.outward_tangent, section))

        for normal, surface_sections in surface_groups:
            prepared = []
            basis_x = None
            for tangent, section in surface_sections:
                planar_tangent = tangent - normal * tangent.dot(normal)
                if planar_tangent.length_squared < 1e-12:
                    continue
                planar_tangent.normalize()
                if basis_x is None:
                    basis_x = planar_tangent.copy()
                prepared.append((planar_tangent, section))
            if len(prepared) < 2 or basis_x is None:
                continue
            basis_y = normal.cross(basis_x).normalized()

            ordered = []
            for tangent, section in prepared:
                angle = atan2(tangent.dot(basis_y), tangent.dot(basis_x))
                oriented_verts = sorted(
                    section.verts,
                    key=lambda vert: normal.dot(
                        tangent.cross(vert.co - spec.core_pos)
                    ),
                )
                spine_index = oriented_verts.index(section.spine_vert)
                ordered.append(
                    (angle, tangent, section, oriented_verts, spine_index)
                )
            ordered.sort(key=lambda item: item[0])

            def add_sector_face(polygon_verts, tangent, section, section_half):
                nonlocal created
                unique_verts = []
                for vert in polygon_verts:
                    if unique_verts and unique_verts[-1] is vert:
                        continue
                    unique_verts.append(vert)
                if len(unique_verts) > 2 and unique_verts[0] is unique_verts[-1]:
                    unique_verts.pop()
                if len(unique_verts) < 3:
                    return
                try:
                    face = bm.faces.new(tuple(unique_verts))
                except ValueError:
                    return
                face.normal_update()
                if face.normal.dot(normal) < 0.0:
                    face.normal_flip()

                sector_y = normal.cross(tangent).normalized()
                spine_uv = next(
                    uv
                    for vert, uv in zip(section.verts, section.uvs)
                    if vert is section.spine_vert
                )
                station_distance = (
                    section.spine_vert.co - spec.core_pos
                ).dot(tangent)
                for loop in face.loops:
                    anchored_uv = next(
                        (
                            uv
                            for vert, uv in zip(section.verts, section.uvs)
                            if vert is loop.vert
                        ),
                        None,
                    )
                    if anchored_uv is not None:
                        loop[uv_layer].uv = anchored_uv
                        continue
                    delta = loop.vert.co - spec.core_pos
                    across = (
                        0.0
                        if wing_width <= 1e-12
                        else delta.dot(sector_y) / wing_width
                    )
                    across = max(-1.0, min(1.0, across))
                    loop[uv_layer].uv = (
                        u_mid + across * u_radius,
                        spine_uv[1]
                        + (delta.dot(tangent) - station_distance)
                        * settings.uv_length_scale,
                    )
                created += 1

                # Junction half-face продолжает соответствующее крыло branch.
                # Сливаем их сразу: отложенный batch dissolve ошибочно считает
                # соседние cross-sections одним region и оставляет прямоугольник.
                for first, second in zip(section_half, section_half[1:]):
                    section_edge = bm.edges.get((first, second))
                    if section_edge is None or len(section_edge.link_faces) != 2:
                        continue
                    bmesh.ops.dissolve_edges(
                        bm,
                        edges=[section_edge],
                        use_verts=False,
                        use_face_split=False,
                    )

            for index, current in enumerate(ordered):
                following = ordered[(index + 1) % len(ordered)]
                angle, tangent, section, section_verts, spine_index = current
                (
                    next_angle,
                    next_tangent,
                    next_section,
                    next_verts,
                    next_spine_index,
                ) = following
                gap = (next_angle - angle) % (2.0 * pi)
                if gap <= 1e-5:
                    continue

                current_half = section_verts[spine_index:]
                next_half = next_verts[: next_spine_index + 1]
                # One-sided CORNER section участвует только в angular gap,
                # куда действительно направлено её крыло. Это важнее выбора
                # кратчайшего gap: owner surface может занимать complement.
                if len(current_half) < 2 or len(next_half) < 2:
                    continue

                current_outer = current_half[-1]
                next_outer = next_half[0]
                divider_pos = _junction_miter_position(
                    spec.core_pos,
                    current_outer.co,
                    tangent,
                    next_outer.co,
                    next_tangent,
                    wing_width,
                )
                if (
                    current_outer.co - divider_pos
                ).length <= DECAL_WELD_DISTANCE:
                    divider_vert = current_outer
                elif (
                    next_outer.co - divider_pos
                ).length <= DECAL_WELD_DISTANCE:
                    divider_vert = next_outer
                else:
                    divider_vert = bm.verts.new(divider_pos)

                # Две half-faces делят сектор по усреднённой линии
                # core -> divider, а не по spine исходной branch.
                add_sector_face(
                    [core_vert, *current_half, divider_vert],
                    tangent,
                    section,
                    current_half,
                )
                add_sector_face(
                    [core_vert, divider_vert, *next_half],
                    next_tangent,
                    next_section,
                    next_half,
                )
    return created


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
    preview=False,
):
    """Manual edge scope: exact physical edges with local owner-side frames."""

    is_seam_mode = mode == "SEAMS"
    width = settings.width_seam if is_seam_mode else settings.width_corner
    uv_rect = DECAL_UV_RECT_SEAM if is_seam_mode else DECAL_UV_RECT_CORNER

    if selected_edge_indices is not None:
        corner_runs, boundary_runs = _collect_manual_edge_decals(
            graph, selected_edge_indices
        )
        if (
            is_seam_mode
            and (corner_runs or boundary_runs)
            and settings.seam_network
        ):
            network_faces = None
            try:
                # Boundary wings — полноправные односторонние сайты сети:
                # divider с соседними seam chains возникает и без общей
                # source-вершины, чисто из конкуренции расстояний.
                network_faces = build_seam_network_faces(
                    corner_runs + boundary_runs,
                    settings.offset,
                    width,
                    preview=preview,
                )
            except Exception as exc:  # непредвиденная геометрия — fallback
                print(
                    "[CFTUV][Decals] Seam network backend failed "
                    f"({exc!r}); falling back to miter pipeline"
                )
            if network_faces:
                _materialize_network_faces(bm, network_faces, settings, uv_rect)
                return
        junction_specs = {}
        junction_cuts = {}
        if is_seam_mode:
            junction_specs, junction_cuts = _prepare_seam_junctions(
                corner_runs, settings, width / 2.0
            )
        junction_ports = []
        for run_index, run in enumerate(corner_runs):
            working_run = run
            if is_seam_mode:
                working_run = _trim_run_for_junctions(
                    run,
                    start_cut=junction_cuts.get((run_index, True), 0.0),
                    end_cut=junction_cuts.get((run_index, False), 0.0),
                )
            is_coplanar = all(
                normal_a.dot(normal_b) > DECAL_COPLANAR_DOT
                for normal_a, normal_b in zip(
                    working_run.segment_normals_a,
                    working_run.segment_normals_b,
                )
            )
            if is_seam_mode and is_coplanar:
                junction_ports.extend(
                    _build_selected_seam_ribbon_run(
                        bm,
                        working_run,
                        settings,
                        uv_rect,
                    )
                )
                continue
            built_ports = _build_corner_ribbon_run(
                bm, working_run, settings, uv_rect, width=width
            )
            if is_seam_mode and built_ports:
                junction_ports.extend(built_ports)
        if is_seam_mode and junction_specs:
            _build_seam_junctions(
                bm,
                junction_specs,
                junction_ports,
                settings,
                uv_rect,
            )
        # Legacy-паритет: посегментные boundary-крылья, как до сети.
        for run in boundary_runs:
            for index in range(len(run.points) - 1):
                _build_boundary_wing_strip(
                    bm,
                    [run.points[index], run.points[index + 1]],
                    run.segment_normals_b[index],
                    settings,
                    uv_rect,
                    closed=False,
                    width=width,
                )
        return

    corner_chains, boundary_chains = _collect_manual_chain_decals(
        graph, chain_refs
    )
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
    preview=False,
):
    if chain_refs is not None and mode in ("CORNERS", "SEAMS"):
        _fill_manual_chain_decals(
            bm,
            graph,
            settings,
            chain_refs,
            mode=mode,
            selected_edge_indices=selected_edge_indices,
            preview=preview,
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
    preview=False,
) -> list[str]:
    """Генерирует decal-объект выбранного режима. Возвращает имена созданных.

    preview=True — быстрый режим для интерактивного modal drag (SEAMS
    network): криволинейные границы не уточняются хордами. Финальная
    генерация вызывается с preview=False.
    """

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
            preview=preview,
        )
    except Exception:
        bm.free()
        raise

    obj = _finalize_decal_object(
        bm, _decal_object_name(mode, source_obj), source_obj, scene
    )
    return [obj.name] if obj is not None else []
