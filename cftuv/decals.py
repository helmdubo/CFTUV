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
  basis_v), собираются в глобальные цепочки через границы patches и
  выдавливаются лентой с адаптивной биссектрисой нормалей на стыках.
- CORNERS: WALL-WALL цепочки с непараллельными нормалями — спайн вдоль
  полилинии цепочки (работает на изогнутых углах) + два крыла по стенам.
- SEAMS: копланарные WALL-WALL цепочки — плоская лента по шву.

UV лент пишутся в прямоугольники атласа (DECAL_UV_RECT_*), продольная
координата — длина дуги * uv_length_scale (final_scale плотности).
"""

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
)
from .model import ChainNeighborKind, ChainRef, DecalSettings, PatchGraph, PatchType

DECAL_MODES = ("TOP", "BOTTOM", "CORNERS", "SEAMS")

_MODE_OBJECT_SUFFIX = {
    "TOP": "Top",
    "BOTTOM": "Bottom",
    "CORNERS": "Corners",
    "SEAMS": "Seams",
}


# ============================================================
# Сбор кандидатов из PatchGraph (чистая геометрия, без bpy)
# ============================================================


def _edge_key(vert_a: int, vert_b: int) -> tuple[int, int]:
    return (vert_a, vert_b) if vert_a < vert_b else (vert_b, vert_a)


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


def _collect_trim_segments(graph: PatchGraph, chain_refs=None):
    """Классифицирует граничные рёбра WALL patches на TOP/BOTTOM.

    outward = edge_dir x patch_normal — направление «наружу» от патча в
    его плоскости (порядок вершин цепочки следует за winding фейсов).
    score = outward . basis_v: > порога — верхняя кромка, < — нижняя.

    Возвращает (top_edges, bottom_edges, edge_frames, vert_cos):
    рёбра — пары индексов вершин, edge_frames — (normal, up) стены на
    ребро, vert_cos — позиции вершин в локальном пространстве.
    """

    top_edges = []
    bottom_edges = []
    edge_frames = {}
    vert_cos = {}

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        if node.patch_type != PatchType.WALL:
            continue
        normal = node.normal
        up = node.basis_v
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                chain_ref = (patch_id, loop_index, chain_index)
                if not _chain_ref_is_enabled(chain_refs, chain_ref):
                    continue
                if not _chain_eligible_for_trim(graph, chain):
                    continue
                verts = chain.vert_indices
                cos = chain.vert_cos
                if len(verts) < 2 or len(verts) != len(cos):
                    continue
                edge_count = len(verts) if chain.is_closed else len(verts) - 1
                for i in range(edge_count):
                    j = (i + 1) % len(verts)
                    seg = cos[j] - cos[i]
                    if seg.length < DECAL_NOISE_THRESHOLD:
                        continue
                    outward = seg.normalized().cross(normal)
                    if outward.length_squared < 1e-8:
                        continue
                    score = outward.normalized().dot(up)
                    if score > DECAL_DIR_THRESHOLD:
                        bucket = top_edges
                    elif score < -DECAL_DIR_THRESHOLD:
                        bucket = bottom_edges
                    else:
                        continue
                    key = _edge_key(verts[i], verts[j])
                    if key in edge_frames:
                        # Ребро уже зарегистрировано другим patch —
                        # non-manifold вход; дубликат сломал бы сборку путей.
                        continue
                    vert_cos[verts[i]] = cos[i]
                    vert_cos[verts[j]] = cos[j]
                    edge_frames[key] = (normal, up)
                    bucket.append((verts[i], verts[j]))

    return top_edges, bottom_edges, edge_frames, vert_cos


def _chain_edge_paths(edges):
    """Собирает рёбра (va, vb) в упорядоченные пути вершин.

    Глобальная сборка: рёбра разных patches с общими вершинами попадают
    в один путь — лента непрерывна через границы patches. Жадный обход
    в обе стороны от стартового ребра (как в прототипе).
    """

    vert_map = {}
    for index, (vert_a, vert_b) in enumerate(edges):
        vert_map.setdefault(vert_a, []).append(index)
        vert_map.setdefault(vert_b, []).append(index)

    visited = set()
    paths = []
    for index, (vert_a, vert_b) in enumerate(edges):
        if index in visited:
            continue
        visited.add(index)
        path = [vert_a, vert_b]

        while True:
            tail = path[-1]
            next_index = next(
                (cand for cand in vert_map[tail] if cand not in visited), None
            )
            if next_index is None:
                break
            visited.add(next_index)
            cand_a, cand_b = edges[next_index]
            path.append(cand_b if cand_a == tail else cand_a)

        while True:
            head = path[0]
            prev_index = next(
                (cand for cand in vert_map[head] if cand not in visited), None
            )
            if prev_index is None:
                break
            visited.add(prev_index)
            cand_a, cand_b = edges[prev_index]
            path.insert(0, cand_b if cand_a == head else cand_a)

        paths.append(path)
    return paths


def _trim_vertex_frames(path, edge_frames):
    """Адаптивный (normal, up) на вершину пути.

    Внутри пути — биссектриса фреймов соседних рёбер (стык двух стен
    получает усреднённое направление), на концах открытого пути — фрейм
    единственного ребра. Замкнутое кольцо (path[0] == path[-1]) получает
    биссектрису и в точке замыкания — дубли вершин совпадут и сварятся.
    """

    is_ring = len(path) > 2 and path[0] == path[-1]
    frames = []
    for i, vert_index in enumerate(path):
        frame_prev = None
        frame_next = None
        if i > 0:
            frame_prev = edge_frames.get(_edge_key(path[i - 1], vert_index))
        elif is_ring:
            frame_prev = edge_frames.get(_edge_key(path[-2], vert_index))
        if i < len(path) - 1:
            frame_next = edge_frames.get(_edge_key(vert_index, path[i + 1]))
        elif is_ring:
            frame_next = edge_frames.get(_edge_key(vert_index, path[1]))

        if frame_prev and frame_next:
            normal = frame_prev[0] + frame_next[0]
            up = frame_prev[1] + frame_next[1]
            if normal.length_squared < 1e-8 or up.length_squared < 1e-8:
                normal, up = frame_prev
            else:
                normal = normal.normalized()
                up = up.normalized()
        elif frame_prev:
            normal, up = frame_prev
        elif frame_next:
            normal, up = frame_next
        else:
            normal, up = Vector((0.0, 0.0, 1.0)), Vector((0.0, 1.0, 0.0))
        frames.append((normal, up))
    return frames


def _collect_wall_pair_chains(graph: PatchGraph, chain_refs=None):
    """WALL-WALL цепочки: (polyline, n_a, n_b, is_closed, convexity) на пару patches.

    Каждая пара обрабатывается один раз — со стороны patch с меньшим id
    (его сегментация цепочек и порядок вершин). Разделение углов и швов
    по dot нормалей patches (порог DECAL_COPLANAR_DOT).
    """

    corner_chains = []
    seam_chains = []
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        if node.patch_type != PatchType.WALL:
            continue
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                chain_ref = (patch_id, loop_index, chain_index)
                if not _chain_ref_is_enabled(chain_refs, chain_ref):
                    continue
                if chain.neighbor_kind != ChainNeighborKind.PATCH:
                    continue
                if chain.neighbor_patch_id <= patch_id:
                    continue
                neighbor = graph.nodes.get(chain.neighbor_patch_id)
                if neighbor is None or neighbor.patch_type != PatchType.WALL:
                    continue
                if len(chain.vert_cos) < 2:
                    continue
                item = (
                    list(chain.vert_cos),
                    node.normal,
                    neighbor.normal,
                    chain.is_closed,
                    chain.dihedral_convexity,
                )
                if node.normal.dot(neighbor.normal) > DECAL_COPLANAR_DOT:
                    seam_chains.append(item)
                else:
                    corner_chains.append(item)
    return corner_chains, seam_chains


def _collect_manual_chain_decals(graph: PatchGraph, chain_refs):
    """Выбранные chains без semantic-фильтра patch type.

    PATCH-neighbor chain даёт угловую декаль по двум поверхностям. MESH_BORDER
    и SEAM_SELF дают плоскую декаль на owner patch. Две стороны одного общего
    seam дедуплицируются по patch pair и исходным mesh edge indices.
    """

    corner_chains = []
    boundary_chains = []
    seen_patch_chains = set()
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

        edge_signature = tuple(sorted(int(index) for index in chain.edge_indices))
        if chain.neighbor_kind == ChainNeighborKind.PATCH:
            neighbor = graph.nodes.get(chain.neighbor_patch_id)
            if neighbor is None:
                continue
            pair_key = (
                min(patch_id, chain.neighbor_patch_id),
                max(patch_id, chain.neighbor_patch_id),
                edge_signature,
            )
            if pair_key in seen_patch_chains:
                continue
            seen_patch_chains.add(pair_key)
            corner_chains.append(
                (
                    list(chain.vert_cos),
                    node.normal,
                    neighbor.normal,
                    chain.is_closed,
                    chain.dihedral_convexity,
                )
            )
            continue

        boundary_key = edge_signature or (patch_id, loop_index, chain_index)
        if boundary_key in seen_boundary_chains:
            continue
        seen_boundary_chains.add(boundary_key)
        boundary_chains.append(
            (list(chain.vert_cos), node.normal, chain.is_closed)
        )

    return corner_chains, boundary_chains


def _collect_manual_flat_chains(graph: PatchGraph, chain_refs):
    """Плоские ленты для выбранных chains без PatchType/coplanarity фильтров."""

    corner_chains, boundary_chains = _collect_manual_chain_decals(
        graph, chain_refs
    )
    flat_chains = [
        (points, normal_a, is_closed)
        for points, normal_a, _normal_b, is_closed, _convexity in corner_chains
    ]
    flat_chains.extend(boundary_chains)
    return flat_chains


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


def _build_trim_strip(bm, path, vert_cos, frames, settings, is_top, uv_rect):
    """Лента трима вдоль пути кромки.

    База — точка кромки + normal * offset, вытяжка вниз (TOP) или вверх
    (BOTTOM) вдоль адаптивного up на height_trim. UV: u — длина дуги,
    v — поперёк прямоугольника атласа.
    """

    if len(path) < 2:
        return
    points = [vert_cos[vert_index] for vert_index in path]
    arc = _arc_lengths(points)

    base_verts = []
    tip_verts = []
    for i, co in enumerate(points):
        normal, up = frames[i]
        extrude_dir = -up if is_top else up
        base_pos = co + normal * settings.offset
        base_verts.append(bm.verts.new(base_pos))
        tip_verts.append(bm.verts.new(base_pos + extrude_dir * settings.height_trim))

    uv_layer = bm.loops.layers.uv.verify()
    u_min, v_min, u_max, v_max = uv_rect
    scale = settings.uv_length_scale
    for i in range(len(points) - 1):
        try:
            face = bm.faces.new(
                (base_verts[i], base_verts[i + 1], tip_verts[i + 1], tip_verts[i])
            )
        except ValueError:
            continue
        u_start = arc[i] * scale
        u_end = arc[i + 1] * scale
        face.loops[0][uv_layer].uv = (u_start, v_min)
        face.loops[1][uv_layer].uv = (u_end, v_min)
        face.loops[2][uv_layer].uv = (u_end, v_max)
        face.loops[3][uv_layer].uv = (u_start, v_max)


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
    half_width = settings.width_corner / 2.0

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


def _build_seam_strip(
    bm, points, normal, settings, uv_rect, closed=False, width=None
):
    """Плоская лента по копланарному шву, центрированная на полилинии."""

    if len(points) < 2:
        return
    offset_vec = normal * settings.offset
    half_width = (settings.width_seam if width is None else width) / 2.0
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


def _fill_decal_bmesh(
    bm,
    graph: PatchGraph,
    settings: DecalSettings,
    mode: str,
    chain_refs=None,
):
    if mode in ("TOP", "BOTTOM"):
        top_edges, bottom_edges, edge_frames, vert_cos = _collect_trim_segments(
            graph, chain_refs=chain_refs
        )
        edges = top_edges if mode == "TOP" else bottom_edges
        uv_rect = DECAL_UV_RECT_TOP if mode == "TOP" else DECAL_UV_RECT_BOTTOM
        for path in _chain_edge_paths(edges):
            frames = _trim_vertex_frames(path, edge_frames)
            _build_trim_strip(
                bm, path, vert_cos, frames, settings, mode == "TOP", uv_rect
            )
    elif mode == "CORNERS":
        if chain_refs is None:
            corner_chains, _seam_chains = _collect_wall_pair_chains(graph)
            boundary_chains = []
        else:
            corner_chains, boundary_chains = _collect_manual_chain_decals(
                graph, chain_refs
            )
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
        for points, normal, is_closed in boundary_chains:
            spine_points = _dedupe_polyline(points)
            _build_seam_strip(
                bm,
                _closed_polyline(spine_points, is_closed),
                normal,
                settings,
                DECAL_UV_RECT_CORNER,
                closed=is_closed,
                width=settings.width_corner,
            )
    elif mode == "SEAMS":
        if chain_refs is None:
            _corner_chains, automatic_seam_chains = _collect_wall_pair_chains(
                graph
            )
            seam_chains = [
                (points, normal_a, is_closed)
                for points, normal_a, _normal_b, is_closed, _convexity
                in automatic_seam_chains
            ]
        else:
            seam_chains = _collect_manual_flat_chains(graph, chain_refs)
        for points, normal, is_closed in seam_chains:
            # Нормаль стороны-владельца (как n_sum_a в прототипе);
            # в manual mode выбранная chain важнее semantic/coplanarity фильтров.
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
        )
    except Exception:
        bm.free()
        raise

    obj = _finalize_decal_object(
        bm, _decal_object_name(mode, source_obj), source_obj, scene
    )
    return [obj.name] if obj is not None else []
