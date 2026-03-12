bl_info = {
    "name": "Hotspot UV + Mesh Decals (Unified Adaptive)",
    "author": "Tech Artist & AI",
    "version": (2, 5, 25),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Hotspot UV",
    "description": "Constraint-First Trim UV: Three-layer (Form/Semantic/Topology) system for trim sheet workflows.",
    "category": "UV",
}

import bpy
import bmesh
import math
import colorsys
from mathutils import Vector
from bpy.props import PointerProperty, IntProperty, FloatProperty, EnumProperty, BoolProperty, StringProperty

# ============================================================
# CONFIGURATION GLOBALS
# ============================================================

TARGET_TEXEL_DENSITY = 512
TEXTURE_SIZE         = 2048
UV_SCALE_MULTIPLIER  = 1.0
FINAL_UV_SCALE       = 0.25
UV_RANGE_LIMIT       = 16.0
WORLD_UP             = Vector((0, 0, 1))

# Boundary edge neighbor types
NB_MESH_BORDER = -1   # край меша (1 face total)
NB_SEAM_SELF   = -2   # seam/sharp внутри патча (обе faces в патче, UV cut)

# ============================================================
# UI SETTINGS
# ============================================================

class HOTSPOTUV_Settings(bpy.types.PropertyGroup):
    target_texel_density: IntProperty(name="Target Texel Density (px/m)", default=512, min=1)
    texture_size: IntProperty(name="Texture Size", default=2048, min=1)
    uv_scale: FloatProperty(name="Custom Scale Multiplier", default=1.0, min=0.0001)
    uv_range_limit: IntProperty(name="UV Range Limit (Tiles)", default=16, min=0)
    # Debug state
    dbg_active: BoolProperty(name="Analyze", default=False, description="Debug analysis mode")
    dbg_source_object: StringProperty(name="Debug Source", default="")
    # Group toggles (expand/collapse + layer visibility)
    dbg_grp_patches: BoolProperty(name="Patches", default=True)
    dbg_grp_frame: BoolProperty(name="Frame", default=True)
    dbg_grp_loops: BoolProperty(name="Loop Types", default=True)
    dbg_grp_overlay: BoolProperty(name="Overlay", default=True)
    # Patches group
    dbg_patches_wall: BoolProperty(name="Wall", default=True)
    dbg_patches_floor: BoolProperty(name="Floor", default=True)
    dbg_patches_slope: BoolProperty(name="Slope", default=True)
    # Frame group
    dbg_frame_h: BoolProperty(name="Horizontal", default=True)
    dbg_frame_v: BoolProperty(name="Vertical", default=True)
    dbg_frame_free: BoolProperty(name="Free", default=True)
    dbg_frame_hole: BoolProperty(name="Holes", default=True)

def _apply_settings_to_globals(settings: HOTSPOTUV_Settings):
    global TARGET_TEXEL_DENSITY, TEXTURE_SIZE, UV_SCALE_MULTIPLIER, FINAL_UV_SCALE, UV_RANGE_LIMIT
    TARGET_TEXEL_DENSITY = int(settings.target_texel_density)
    TEXTURE_SIZE         = int(settings.texture_size)
    UV_SCALE_MULTIPLIER  = float(settings.uv_scale)
    UV_RANGE_LIMIT       = float(settings.uv_range_limit)
    FINAL_UV_SCALE = (TARGET_TEXEL_DENSITY / TEXTURE_SIZE) * UV_SCALE_MULTIPLIER

# ============================================================
# GEOMETRY ANALYSIS
# ============================================================

class IslandInfo:
    def __init__(self, faces, index):
        self.faces = faces
        self.index = index
        self.avg_normal = Vector((0, 0, 1))
        self.type = "WALL"
        self.area = 0.0
        self.perimeter = 0.0

def get_expanded_islands(bm, initial_faces):
    """
    Новая функция: находит весь лоскут (до sharp edges), 
    но разделяет его на 'full' (все полигоны) и 'core' (только те, что выделил юзер).
    """
    visited = set()
    islands = []
    initial_set = set(initial_faces)

    for start_f in initial_faces:
        if start_f in visited: continue

        full_faces = []
        core_faces = []
        stack = [start_f]
        visited.add(start_f)

        while stack:
            curr = stack.pop()
            full_faces.append(curr)
            if curr in initial_set:
                core_faces.append(curr)

            for edge in curr.edges:
                if not edge.smooth or edge.seam: continue
                for neighbor in edge.link_faces:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)

        islands.append({
            'full': full_faces,
            'core': core_faces
        })

    return islands

def find_island_up(island):
    """
    Dual-Strategy: ищем ориентацию лоскута двумя путями одновременно.
    
    Стратегия 1 (Direct Up): классический поиск рёбер, близких к WORLD_UP.
    Стратегия 2 (Derived Up): поиск доминирующих горизонтальных рёбер,
        затем вывод up через cross(normal, right).
    
    Побеждает стратегия с более сильным сигналом. Это решает проблему стен
    с зигзагообразными торцами, где горизонтальные рёбра основания дают
    чистую ориентацию, а вертикальных прямых рёбер нет.
    """
    edge_dirs = {}
    for f in island.faces:
        for e in f.edges:
            length = e.calc_length()
            if length < 1e-4: continue
            vec = (e.verts[1].co - e.verts[0].co).normalized()
            if vec.dot(WORLD_UP) < 0: vec = -vec
            qx, qy, qz = round(vec.x, 2), round(vec.y, 2), round(vec.z, 2)
            key = (qx, qy, qz)
            if key not in edge_dirs: edge_dirs[key] = {'vec': vec, 'weight': 0.0}
            edge_dirs[key]['weight'] += length

    # --- Стратегия 1: Direct Up (ищем вертикальные рёбра) ---
    best_direct_up = WORLD_UP
    max_up_score = -1.0
    for data in edge_dirs.values():
        vec = data['vec']
        alignment = abs(vec.dot(WORLD_UP))
        score = data['weight'] * (alignment ** 2)
        if score > max_up_score:
            max_up_score = score
            best_direct_up = vec

    # --- Стратегия 2: Derived Up (ищем горизонтальные рёбра → выводим up) ---
    best_right = None
    max_right_score = -1.0
    for data in edge_dirs.values():
        vec = data['vec']
        horizontal = 1.0 - abs(vec.dot(WORLD_UP))
        score = data['weight'] * (horizontal ** 2)
        if score > max_right_score:
            max_right_score = score
            best_right = vec

    # --- Выбираем победителя ---
    if max_right_score > max_up_score and best_right is not None:
        # Горизонтальные рёбра доминируют → выводим up из cross(normal, right)
        normal = island.avg_normal
        derived_up = normal.cross(best_right)
        # Гарантируем что derived_up смотрит вверх
        if derived_up.dot(WORLD_UP) < 0:
            derived_up = -derived_up
        if derived_up.length_squared > 1e-6:
            return derived_up.normalized()

    # Direct Up победил или derived не дал результата
    return best_direct_up.normalized() if max_up_score > 0 else WORLD_UP

def analyze_island_properties(island_obj):
    avg_n = Vector((0, 0, 0))
    total_area = 0.0
    perimeter = 0.0
    island_faces_set = set(island_obj.faces)
    
    for f in island_obj.faces:
        f_area = f.calc_area()
        avg_n += f.normal * f_area
        total_area += f_area
        for e in f.edges:
            link_count = sum(1 for lf in e.link_faces if lf in island_faces_set)
            if link_count == 1: perimeter += e.calc_length()

    if avg_n.length > 0: avg_n.normalize()
    else: avg_n = Vector((0, 0, 1))
    
    island_obj.avg_normal = avg_n
    island_obj.area = total_area
    island_obj.perimeter = perimeter
    up_dot = abs(avg_n.dot(WORLD_UP))
    if up_dot > 0.9:
        island_obj.type = "FLOOR"
    elif up_dot < 0.3:
        island_obj.type = "WALL"
    else:
        island_obj.type = "SLOPE"

def build_edge_based_links(islands, bm):
    edge_to_islands = {}
    for isl in islands:
        for f in isl.faces:
            for e in f.edges:
                edge_to_islands.setdefault(e.index, set()).add(isl.index)

    links_dict = {}
    for e_idx, isl_indices in edge_to_islands.items():
        if len(isl_indices) == 2:
            i_a, i_b = list(isl_indices)
            if i_a > i_b: i_a, i_b = i_b, i_a
            pair = (i_a, i_b)
            edge_len = bm.edges[e_idx].calc_length()
            if pair not in links_dict:
                v1, v2 = bm.edges[e_idx].verts[0].index, bm.edges[e_idx].verts[1].index
                links_dict[pair] = {
                    'shared_length': 0.0,
                    'shared_verts': set(),
                    'longest_edge_len': edge_len,
                    'longest_edge_verts': [v1, v2]
                }
            links_dict[pair]['shared_length'] += edge_len
            links_dict[pair]['shared_verts'].add(bm.edges[e_idx].verts[0].index)
            links_dict[pair]['shared_verts'].add(bm.edges[e_idx].verts[1].index)
            # Обновляем на самое длинное ребро — оно даёт самый точный угол
            if edge_len > links_dict[pair]['longest_edge_len']:
                links_dict[pair]['longest_edge_len'] = edge_len
                v1, v2 = bm.edges[e_idx].verts[0].index, bm.edges[e_idx].verts[1].index
                links_dict[pair]['longest_edge_verts'] = [v1, v2]
            
    return [{'isl_a': k[0], 'isl_b': k[1], 
             'shared_length': v['shared_length'], 
             'shared_verts': list(v['shared_verts']),
             'longest_edge_verts': v['longest_edge_verts']} for k, v in links_dict.items()]

def calc_surface_basis(normal, ref_up=WORLD_UP):
    up_proj = ref_up - normal * ref_up.dot(normal)
    if up_proj.length_squared < 1e-5:
        tangent = Vector((1, 0, 0))
        tangent = (tangent - normal * tangent.dot(normal)).normalized()
        return tangent, normal.cross(tangent).normalized()
    bitangent = up_proj.normalized()
    return bitangent.cross(normal).normalized(), bitangent

# ============================================================
# PATCH & FRAME ANALYSIS (Iteration 0-2)
# ============================================================

def find_seam_patches(bm, base_faces):
    """
    Flood fill лоскутов, разделённых seam/sharp edges.
    Аналог get_expanded_islands, но без разделения на core/full.
    Возвращает: [[BMFace, ...], ...]
    """
    face_set = set(base_faces)
    visited = set()
    patches = []
    
    for f0 in base_faces:
        if f0 in visited:
            continue
        stack = [f0]
        visited.add(f0)
        patch = []
        while stack:
            f = stack.pop()
            patch.append(f)
            for e in f.edges:
                if e.seam or not e.smooth:
                    continue
                for nf in e.link_faces:
                    if nf in face_set and nf not in visited:
                        visited.add(nf)
                        stack.append(nf)
        patches.append(patch)
    return patches


def find_patch_boundary_edges(patch_faces):
    """
    Boundary edge патча — edge где UV лоскута разрывается:
      1. in_count == 1 — inter-patch boundary или mesh border
      2. in_count == 2 AND (seam OR NOT smooth) — seam/sharp внутри патча (UV cut)
    
    Возвращает list of BMEdge.
    """
    patch_set = set(patch_faces)
    seen = set()
    boundary = []
    for f in patch_faces:
        for e in f.edges:
            if e.index in seen:
                continue
            seen.add(e.index)
            in_count = sum(1 for lf in e.link_faces if lf in patch_set)
            if in_count == 1:
                boundary.append(e)
            elif in_count == 2 and (e.seam or not e.smooth):
                # SEAM_SELF: обе faces в патче, но seam/sharp разрезает UV
                boundary.append(e)
    return boundary


def find_boundary_edge_neighbors(boundary_edges, patch_faces, face_to_patch, patch_idx):
    """
    Step 1: Для каждого boundary edge определяет тип соседа.
    
    Neighbor types:
      int >= 0     — PATCH_NEIGHBOR (соседний патч по index)
      NB_MESH_BORDER (-1) — край меша (1 face total, дыра/открытый край)
      NB_SEAM_SELF   (-2) — seam/sharp внутри патча (обе faces в патче)
    
    Возвращает: dict {edge.index: neighbor_id}
    """
    patch_set = set(f.index for f in patch_faces)
    edge_neighbor = {}
    
    for e in boundary_edges:
        if len(e.link_faces) == 1:
            edge_neighbor[e.index] = NB_MESH_BORDER
        else:
            faces_in_patch = sum(1 for lf in e.link_faces if lf.index in patch_set)
            if faces_in_patch == 2:
                # Both faces in same patch → SEAM_SELF
                edge_neighbor[e.index] = NB_SEAM_SELF
            else:
                # One face in patch, other in different patch
                other_faces = [lf for lf in e.link_faces if lf.index not in patch_set]
                if other_faces:
                    edge_neighbor[e.index] = face_to_patch.get(other_faces[0].index, NB_MESH_BORDER)
                else:
                    edge_neighbor[e.index] = NB_MESH_BORDER
    
    return edge_neighbor


def split_loop_into_chains_by_neighbor(loop_verts, loop_edges, edge_neighbor):
    """
    Step 2: Разбивает замкнутый boundary loop на chains по смене соседа.
    
    Chain = максимальная последовательность boundary edges с одним и тем же neighbor.
    Split point = vertex где neighbor меняется.
    
    Возвращает: [{'verts': [BMVert], 'edges': [BMEdge], 'neighbor': int, 'is_closed': bool}]
    """
    n = len(loop_verts)
    ne = len(loop_edges)
    if ne == 0:
        return []
    
    # edge[i] connects loop_verts[i] → loop_verts[(i+1)%n]
    nbs = [edge_neighbor.get(e.index, -1) for e in loop_edges]
    
    # Find split vertices: where neighbor changes
    # Vertex i is split if nb of edge[i-1] != nb of edge[i]
    split_indices = []
    for i in range(n):
        if nbs[(i - 1) % ne] != nbs[i % ne]:
            split_indices.append(i)
    
    if not split_indices:
        # Entire loop = one chain (closed, single neighbor)
        return [{'verts': list(loop_verts), 'edges': list(loop_edges),
                 'neighbor': nbs[0], 'is_closed': True}]
    
    chains = []
    num_splits = len(split_indices)
    for si in range(num_splits):
        v_start = split_indices[si]
        v_end = split_indices[(si + 1) % num_splits]
        
        chain_verts = []
        chain_edges = []
        idx = v_start
        safety = 0
        while safety < n + 2:
            safety += 1
            chain_verts.append(loop_verts[idx % n])
            chain_edges.append(loop_edges[idx % ne])
            idx += 1
            if idx % n == v_end % n:
                chain_verts.append(loop_verts[v_end % n])  # end vertex
                break
        
        chains.append({
            'verts': chain_verts,
            'edges': chain_edges,
            'neighbor': nbs[v_start % ne],
            'is_closed': False,
        })
    
    return chains


def classify_loops_via_uv(loops, patch_faces, uv_layer):
    """
    Step 4: Классификация OUTER/HOLE через анализ вложенности (nesting) 
    в 2D пространстве временной UV развертки.
    """
    if not loops:
        return
    
    if len(loops) == 1:
        loops[0]['kind'] = 'OUTER'
        loops[0]['depth'] = 0
        return
    
    patch_faces_set = set(patch_faces)
    
    def get_vert_uv(v):
        # Усредняем UV всех лупов этой вершины внутри патча.
        # Это "схлопывает" внутренние швы (SEAM_SELF) в одну линию в 2D,
        # сохраняя контур замкнутым без потери данных о вложенности.
        uvs = [l[uv_layer].uv for l in v.link_loops if l.face in patch_faces_set]
        if uvs:
            return (sum(uv.x for uv in uvs) / len(uvs), sum(uv.y for uv in uvs) / len(uvs))
        return (0.0, 0.0)
    
    # 1. Проецируем loops в 2D (UV)
    polys_2d = []
    for lp in loops:
        poly = [get_vert_uv(v) for v in lp['verts']]
        polys_2d.append(poly)
    
    # 2. Математика 2D
    def signed_area(poly):
        s = 0.0
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            s += x1 * y2 - x2 * y1
        return 0.5 * s
    
    def point_in_poly(pt, poly):
        x, y = pt
        inside = False
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                x_int = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-30) + x1
                if x < x_int:
                    inside = not inside
        return inside
    
    def interior_point(poly):
        n = len(poly)
        if n < 3:
            return poly[0] if poly else (0.0, 0.0)
        
        sa = signed_area(poly)
        edges_by_len = []
        for i in range(n):
            j = (i + 1) % n
            dx = poly[j][0] - poly[i][0]
            dy = poly[j][1] - poly[i][1]
            edges_by_len.append((dx * dx + dy * dy, i))
        edges_by_len.sort(reverse=True)
        
        for len2, i in edges_by_len:
            j = (i + 1) % n
            mx = (poly[i][0] + poly[j][0]) * 0.5
            my = (poly[i][1] + poly[j][1]) * 0.5
            dx = poly[j][0] - poly[i][0]
            dy = poly[j][1] - poly[i][1]
            elen = math.sqrt(len2)
            if elen < 1e-12:
                continue
            
            if sa >= 0:
                nx, ny = -dy / elen, dx / elen
            else:
                nx, ny = dy / elen, -dx / elen
            
            eps = elen * 0.01
            pt = (mx + nx * eps, my + ny * eps)
            if point_in_poly(pt, poly):
                return pt
                
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    
    # 3. Вычисляем вложенность (nesting depth)
    int_pts = [interior_point(p) for p in polys_2d]
    
    for i, lp in enumerate(loops):
        depth = 0
        for j, poly in enumerate(polys_2d):
            if i == j:
                continue
            if point_in_poly(int_pts[i], poly):
                depth += 1
        
        lp['depth'] = depth
        lp['area_uv'] = signed_area(polys_2d[i])
        
        # depth == 0 (наружный контур) или четная вложенность (кольцо внутри кольца)
        if depth == 0 or depth % 2 == 0:
            lp['kind'] = 'OUTER'
        else:
            lp['kind'] = 'HOLE'


def build_ordered_boundary_loops(boundary_edges):
    """
    Собирает boundary edges в упорядоченные замкнутые loops.
    Возвращает: [{'verts': [BMVert, ...], 'edges': [BMEdge, ...]}, ...]
    """
    v2e = {}
    for e in boundary_edges:
        for v in e.verts:
            v2e.setdefault(v, []).append(e)
    
    used = set()
    loops = []
    
    for e0 in boundary_edges:
        if e0 in used:
            continue
        v0 = e0.verts[0]
        used.add(e0)
        verts = [v0, e0.other_vert(v0)]
        edges = [e0]
        
        curr_v = verts[-1]
        safety = 0
        while safety < 200000:
            safety += 1
            cand = [e for e in v2e.get(curr_v, []) if e not in used]
            if not cand:
                break
            e_next = cand[0]
            used.add(e_next)
            v_next = e_next.other_vert(curr_v)
            edges.append(e_next)
            if v_next == v0:
                loops.append({'verts': verts, 'edges': edges})
                break
            verts.append(v_next)
            curr_v = v_next
    
    return loops

def build_patch_basis(patch_faces):
    """
    Строит полный локальный базис для патча.
    Возвращает: (centroid, normal, seed_t, seed_b, island_type)
    
    seed_t = горизонталь (U direction)
    seed_b = вертикаль (V direction)
    normal = нормаль патча
    """
    # Используем существующие функции
    temp_island = IslandInfo(patch_faces, 0)
    analyze_island_properties(temp_island)
    
    island_up = find_island_up(temp_island)
    
    # Для WALL: детерминистичный базис через avg_normal + WORLD_UP.
    # find_island_up + calc_surface_basis(seed_face) ненадёжны на single-face
    # вытянутых quad-ах — seed_t/seed_b могут быть перепутаны.
    # Прямая проекция WORLD_UP на плоскость патча даёт однозначный результат.
    if temp_island.type in ("WALL", "SLOPE"):
        n = temp_island.avg_normal
        up_proj = WORLD_UP - n * WORLD_UP.dot(n)
        if up_proj.length_squared > 1e-8:
            seed_b = up_proj.normalized()          # V = vertical on wall
            seed_t = seed_b.cross(n).normalized()  # U = horizontal on wall
        else:
            # Дегенерат: нормаль ≈ WORLD_UP → это скорее FLOOR, но type сказал WALL
            seed_t, seed_b = calc_surface_basis(temp_island.avg_normal, island_up)
    else:
        # FLOOR: оригинальная логика через seed_face
        sorted_faces = sorted(
            patch_faces,
            key=lambda f: f.calc_area() * (max(0, f.normal.dot(temp_island.avg_normal)) ** 4),
            reverse=True
        )
        seed_face = sorted_faces[0] if sorted_faces else patch_faces[0]
        seed_t, seed_b = calc_surface_basis(seed_face.normal, island_up)
    
    # Центроид
    center = Vector((0, 0, 0))
    cnt = 0
    for f in patch_faces:
        for v in f.verts:
            center += v.co
            cnt += 1
    centroid = center / max(cnt, 1)
    
    return centroid, temp_island.avg_normal, seed_t, seed_b, temp_island.type


def find_loop_corners(loop_verts, angle_threshold_deg=30.0):
    """
    Находит угловые вершины boundary loop — точки, где цепочка
    резко поворачивает. Разбивает loop на сегменты по этим точкам.
    
    angle_threshold_deg: минимальный угол отклонения ОТ ПРЯМОЙ для corner.
      30° → ловит бевели (45° поворот) и прямые углы (90°).
    
    Возвращает: list of corner indices (в пределах loop_verts)
    """
    n = len(loop_verts)
    if n < 3:
        return []
    
    # cos(30°) = 0.866 → ловит повороты > 30° от прямой
    cos_threshold = math.cos(math.radians(angle_threshold_deg))
    corners = []
    
    for i in range(n):
        v_prev = loop_verts[(i - 1) % n].co
        v_curr = loop_verts[i].co
        v_next = loop_verts[(i + 1) % n].co
        
        d1 = (v_curr - v_prev)
        d2 = (v_next - v_curr)
        
        if d1.length < 1e-8 or d2.length < 1e-8:
            corners.append(i)
            continue
        
        cos_angle = d1.normalized().dot(d2.normalized())
        # cos_angle ≈ 1.0 = прямая, ≈ 0 = 90° поворот, < 0 = разворот
        # Ловим когда cos_angle < cos_threshold (поворот больше threshold)
        if cos_angle < cos_threshold:
            corners.append(i)
    
    return corners


def split_loop_into_segments(loop_verts, corners):
    """
    Разбивает замкнутый loop на сегменты по corner вершинам.
    Каждый сегмент включает corner на обоих концах.
    
    Возвращает: [[BMVert, ...], ...]
    """
    n = len(loop_verts)
    
    if not corners:
        return [loop_verts]  # Весь loop — один сегмент
    
    segments = []
    num_corners = len(corners)
    
    for ci in range(num_corners):
        start_idx = corners[ci]
        end_idx = corners[(ci + 1) % num_corners]
        
        seg = []
        idx = start_idx
        while True:
            seg.append(loop_verts[idx % n])
            if idx % n == end_idx % n:
                break
            idx += 1
            if len(seg) > n + 1:
                break
        
        if len(seg) >= 2:
            segments.append(seg)
    
    return segments


def classify_segment_frame_role(segment_verts, seed_t, seed_b, threshold=0.08):
    """
    Определяет роль СЕГМЕНТА boundary loop для frame.
    
    H_FRAME = горизонтальная линия в 3D (low V variance), straighten по V на UV
    V_FRAME = вертикальная линия в 3D (low U variance), straighten по U на UV
    FREE = диагональная или слишком короткая
    """
    if len(segment_verts) < 2:
        return 'FREE'
    
    us = [v.co.dot(seed_t) for v in segment_verts]
    vs = [v.co.dot(seed_b) for v in segment_verts]
    
    extent_u = max(us) - min(us)
    extent_v = max(vs) - min(vs)
    
    total_extent = max(extent_u, extent_v)
    if total_extent < 1e-6:
        return 'FREE'
    
    ratio_v = extent_v / total_extent
    ratio_u = extent_u / total_extent
    
    if ratio_v < threshold:
        return 'H_FRAME'
    if ratio_u < threshold:
        return 'V_FRAME'
    
    return 'FREE'


def analyze_all_patches(bm, base_faces, obj=None):
    """
    Полный анализ: патчи, boundary chains, loops, OUTER/HOLE, frame classification.
    """
    patches = find_seam_patches(bm, base_faces)
    
    # Face → patch index map
    face_to_patch = {}
    for pi, pf in enumerate(patches):
        for f in pf:
            face_to_patch[f.index] = pi
            
    # --- Setup Temporary UV Layer ---
    uv_temp_name = "_cftuv_temp"
    uv_layer = bm.loops.layers.uv.get(uv_temp_name)
    if not uv_layer:
        uv_layer = bm.loops.layers.uv.new(uv_temp_name)
        
    original_active_uv_name = None
    if obj and obj.type == 'MESH':
        bmesh.update_edit_mesh(obj.data)
        if obj.data.uv_layers.active:
            original_active_uv_name = obj.data.uv_layers.active.name
        # Делаем наш временный слой активным для unwrap
        obj.data.uv_layers[uv_temp_name].active = True
    
    original_selection = [f for f in bm.faces if f.select]
    for f in bm.faces:
        f.select = False
    
    results = []
    for pi, patch_faces in enumerate(patches):
        # --- Unwrap this patch ---
        for f in patch_faces:
            f.select = True
        
        if obj and obj.type == 'MESH':
            bmesh.update_edit_mesh(obj.data)
            bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)
            
        for f in patch_faces:
            f.select = False
            
        # --- Boundary edges + neighbor info ---
        be = find_patch_boundary_edges(patch_faces)
        edge_neighbor = find_boundary_edge_neighbors(be, patch_faces, face_to_patch, pi)
        
        # --- Ordered loops (closed contours) ---
        raw_loops = build_ordered_boundary_loops(be)
        
        # --- Classify OUTER/HOLE via UV ---
        classify_loops_via_uv(raw_loops, patch_faces, uv_layer)
        
        # --- Split loops into chains by neighbor ---
        all_chains = []
        for lp in raw_loops:
            chains = split_loop_into_chains_by_neighbor(lp['verts'], lp['edges'], edge_neighbor)
            lp['chains'] = chains
            all_chains.extend(chains)
        
        # --- Basis ---
        centroid, normal, seed_t, seed_b, isl_type = build_patch_basis(patch_faces)
        
        # --- Serialize loop/chain vertex coords (survive mode switch) ---
        for lp in raw_loops:
            lp['vert_cos'] = [v.co.copy() for v in lp['verts']]
            for chain in lp.get('chains', []):
                chain['vert_cos'] = [v.co.copy() for v in chain['verts']]
        
        # --- Corner detection + frame classification on OUTER loop segments ---
        all_segments = []
        for lp in raw_loops:
            corners = find_loop_corners(lp['verts'])
            segments = split_loop_into_segments(lp['verts'], corners)
            
            lp_segments = []
            for seg_verts in segments:
                role = classify_segment_frame_role(seg_verts, seed_t, seed_b)
                lp_segments.append({
                    'vert_cos': [v.co.copy() for v in seg_verts],
                    'frame_role': role,
                    'loop_kind': lp.get('kind', 'OUTER')
                })
            
            lp['segments'] = lp_segments
            all_segments.extend(lp_segments)
        
        # --- Serialize mesh faces for debug fill ---
        vert_map = {}
        mesh_verts = []
        mesh_faces = []
        for f in patch_faces:
            face_indices = []
            for v in f.verts:
                if v.index not in vert_map:
                    vert_map[v.index] = len(mesh_verts)
                    mesh_verts.append(v.co.copy())
                face_indices.append(vert_map[v.index])
            if len(face_indices) == 3:
                mesh_faces.append(tuple(face_indices))
            else:
                for i in range(1, len(face_indices) - 1):
                    mesh_faces.append((face_indices[0], face_indices[i], face_indices[i + 1]))
        
        results.append({
            'faces': patch_faces,
            'centroid': centroid,
            'normal': normal,
            'seed_t': seed_t,
            'seed_b': seed_b,
            'type': isl_type,
            'loops': raw_loops,
            'all_chains': [{'vert_cos': c['vert_cos'], 'neighbor': c['neighbor'],
                            'is_closed': c.get('is_closed', False)} for c in all_chains],
            'all_segments': all_segments,
            'mesh_verts': mesh_verts,
            'mesh_faces': mesh_faces,
        })
        
    # --- Cleanup Temporary UV Layer & Restore Selection ---
    bm.loops.layers.uv.remove(uv_layer)
    if obj and obj.type == 'MESH':
        bmesh.update_edit_mesh(obj.data) # Синхронизируем удаление слоя с объектом
        if original_active_uv_name and original_active_uv_name in obj.data.uv_layers:
            obj.data.uv_layers[original_active_uv_name].active = True
        
    for f in original_selection:
        f.select = True
        
    if obj and obj.type == 'MESH':
        bmesh.update_edit_mesh(obj.data)
        
    return results

# ============================================================
# DEBUG VISUALIZATION (Grease Pencil)
# ============================================================

GP_DEBUG_PREFIX = "CFTUV_Debug_"

# Layer names and RGBA colors — naming maps to panel groups
_GP_STYLES = {
    # Frame group
    'Frame_H':         (1.0, 0.85, 0.0, 1.0),    # yellow — horizontal
    'Frame_V':         (0.0, 0.85, 0.85, 1.0),    # cyan — vertical
    'Frame_FREE':      (0.5, 0.5, 0.5, 0.6),      # gray
    'Frame_HOLE':      (0.2, 0.2, 0.6, 0.8),      # dark blue
    # Patches layers are created dynamically per type: Patches_WALL, Patches_FLOOR, Patches_SLOPE
}

# Basis axis colors (drawn into single Overlay_Basis layer)
_BASIS_COLORS = {
    'U': (1.0, 0.15, 0.15, 1.0),    # red
    'V': (0.15, 1.0, 0.15, 1.0),    # green
    'N': (0.2, 0.2, 1.0, 1.0),      # blue
}


def _get_gp_debug_name(source_obj):
    return GP_DEBUG_PREFIX + source_obj.name


def _get_or_create_gp_object(source_obj):
    """Находит или создаёт GP объект для debug визуализации."""
    gp_name = _get_gp_debug_name(source_obj)
    
    if gp_name in bpy.data.objects:
        gp_obj = bpy.data.objects[gp_name]
        if gp_obj.type == 'GPENCIL':
            return gp_obj
        bpy.data.objects.remove(gp_obj, do_unlink=True)
    
    gp_data = bpy.data.grease_pencils.new(gp_name)
    gp_obj = bpy.data.objects.new(gp_name, gp_data)
    bpy.context.scene.collection.objects.link(gp_obj)
    
    # Привязываем к тому же transform что и source
    gp_obj.matrix_world = source_obj.matrix_world.copy()
    
    # GP display defaults
    gp_data.stroke_depth_order = '3D'             # Stroke Depth Order = 3D Location
    gp_data.stroke_thickness_space = 'SCREENSPACE'  # Stroke Thickness = Screen Space
    
    return gp_obj


def _ensure_gp_layer(gp_data, layer_name, color_rgba):
    """Создаёт или очищает GP layer + material."""
    # Material
    mat_name = f"CFTUV_{layer_name}"
    if mat_name in bpy.data.materials:
        mat = bpy.data.materials[mat_name]
    else:
        mat = bpy.data.materials.new(mat_name)
        bpy.data.materials.create_gpencil_data(mat)
    
    mat.grease_pencil.color = color_rgba[:4]
    mat.grease_pencil.show_fill = False
    
    # Ensure material is on gp_data
    mat_idx = None
    for i, slot in enumerate(gp_data.materials):
        if slot and slot.name == mat_name:
            mat_idx = i
            break
    if mat_idx is None:
        gp_data.materials.append(mat)
        mat_idx = len(gp_data.materials) - 1
    
    # Layer
    if layer_name in gp_data.layers:
        layer = gp_data.layers[layer_name]
        layer.clear()
    else:
        layer = gp_data.layers.new(layer_name, set_active=False)
    
    # Ensure frame 0
    if not layer.frames:
        frame = layer.frames.new(0)
    else:
        frame = layer.frames[0]
    
    return frame, mat_idx


def _add_gp_stroke(frame, points, mat_idx, line_width=4):
    """Добавляет stroke из списка Vector точек (local space)."""
    if len(points) < 2:
        return
    stroke = frame.strokes.new()
    stroke.material_index = mat_idx
    stroke.line_width = line_width
    stroke.points.add(len(points))
    for i, p in enumerate(points):
        stroke.points[i].co = (p.x, p.y, p.z)
        stroke.points[i].strength = 1.0
        stroke.points[i].pressure = 1.0


def _clear_gp_debug(source_obj):
    """Удаляет GP debug объект, patch mesh и per-patch materials."""
    gp_name = _get_gp_debug_name(source_obj)
    if gp_name in bpy.data.objects:
        obj = bpy.data.objects[gp_name]
        bpy.data.objects.remove(obj, do_unlink=True)
    if gp_name in bpy.data.grease_pencils:
        bpy.data.grease_pencils.remove(bpy.data.grease_pencils[gp_name])
    # Cleanup patch mesh
    mesh_name = GP_DEBUG_PREFIX + "Mesh_" + source_obj.name
    if mesh_name in bpy.data.objects:
        obj = bpy.data.objects[mesh_name]
        mesh_data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh_data and mesh_data.users == 0:
            bpy.data.meshes.remove(mesh_data)
    # Cleanup per-patch materials
    for mat in list(bpy.data.materials):
        if (mat.name.startswith("CFTUV_P") or mat.name.startswith("CFTUV_Axis_") or
            mat.name.startswith("CFTUV_Ch") or mat.name.startswith("CFTUV_Lp") or
            mat.name.startswith("CFTUV_Hl")) and mat.users == 0:
            bpy.data.materials.remove(mat)


def _create_patch_mesh(patch_results, source_obj):
    """Создаёт mesh объект с per-patch materials для визуализации патчей.
    Concave faces и holes отображаются корректно (реальная геометрия)."""
    mesh_name = GP_DEBUG_PREFIX + "Mesh_" + source_obj.name
    
    # Удаляем старый если есть
    if mesh_name in bpy.data.objects:
        old_obj = bpy.data.objects[mesh_name]
        old_data = old_obj.data
        bpy.data.objects.remove(old_obj, do_unlink=True)
        if old_data and old_data.users == 0:
            bpy.data.meshes.remove(old_data)
    
    # Собираем все вершины и faces с offset per patch
    all_verts = []
    all_faces = []
    face_mat_indices = []
    golden_ratio = 0.618033988749895
    materials = []
    
    for pi, patch in enumerate(patch_results):
        offset = len(all_verts)
        
        for v in patch['mesh_verts']:
            all_verts.append(v)
        
        for f_indices in patch['mesh_faces']:
            all_faces.append(tuple(idx + offset for idx in f_indices))
            face_mat_indices.append(pi)
        
        # Material
        hue = (pi * golden_ratio) % 1.0
        if patch['type'] == 'WALL':
            sat, val, alpha = 0.8, 0.9, 0.4
        else:
            sat, val, alpha = 0.5, 0.6, 0.3
        r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
        
        mat_name = f"CFTUV_P{pi:03d}"
        if mat_name in bpy.data.materials:
            mat = bpy.data.materials[mat_name]
        else:
            mat = bpy.data.materials.new(mat_name)
        mat.diffuse_color = (r, g, b, alpha)
        mat.use_nodes = False
        materials.append(mat)
    
    # Создаём mesh
    mesh_data = bpy.data.meshes.new(mesh_name)
    mesh_data.from_pydata([v[:] for v in all_verts], [], all_faces)
    mesh_data.update()
    
    # Назначаем materials
    for mat in materials:
        mesh_data.materials.append(mat)
    
    for fi, mat_idx in enumerate(face_mat_indices):
        if fi < len(mesh_data.polygons):
            mesh_data.polygons[fi].material_index = mat_idx
    
    # Создаём object
    mesh_obj = bpy.data.objects.new(mesh_name, mesh_data)
    mesh_obj.matrix_world = source_obj.matrix_world.copy()
    bpy.context.scene.collection.objects.link(mesh_obj)
    
    # Display settings — скрыт по умолчанию, backup для инспекции
    mesh_obj.show_transparent = True
    mesh_obj.display_type = 'SOLID'
    mesh_obj.color = (1, 1, 1, 0.5)
    mesh_obj.hide_viewport = True
    mesh_obj.hide_render = True


def _bridge_outer_with_holes(outer_cos, holes_cos):
    """Превращает polygon-with-holes в simple polygon через bridge edges.
    
    Алгоритм: для каждого hole находим ближайшую пару вершин (outer↔hole),
    вставляем bridge (дублированные рёбра туда-обратно), получаем один контур.
    
    Returns: list of Vector — единый замкнутый контур.
    """
    if not holes_cos:
        return list(outer_cos)
    
    contour = list(outer_cos)
    
    for hole in holes_cos:
        if len(hole) < 3:
            continue
        
        # Ищем ближайшую пару (contour_vertex, hole_vertex)
        best_dist = float('inf')
        best_ci = 0
        best_hi = 0
        for ci, cv in enumerate(contour):
            for hi, hv in enumerate(hole):
                d = (cv - hv).length_squared
                if d < best_dist:
                    best_dist = d
                    best_ci = ci
                    best_hi = hi
        
        # Строим bridge: contour[...ci] → hole[hi...hi(wrapped)] → contour[ci...]
        # Hole обходим в обратном направлении (CW если outer CCW)
        hole_reordered = []
        n_hole = len(hole)
        for k in range(n_hole + 1):  # +1 чтобы замкнуть hole
            hole_reordered.append(hole[(best_hi - k) % n_hole])
        
        new_contour = contour[:best_ci + 1] + hole_reordered + contour[best_ci:]
        contour = new_contour
    
    return contour


def create_debug_visualization(patch_results, source_obj, dbg_settings=None):
    """Создаёт GP strokes для визуализации: Patches (per type), Frame, Overlay."""
    gp_obj = _get_or_create_gp_object(source_obj)
    gp_data = gp_obj.data
    
    # Mesh backup (hidden)
    _create_patch_mesh(patch_results, source_obj)
    
    # --- Frame layers (static styles) ---
    frames_and_mats = {}
    for style_name, color in _GP_STYLES.items():
        frame, mat_idx = _ensure_gp_layer(gp_data, style_name, color)
        frames_and_mats[style_name] = (frame, mat_idx)
    
    # --- Overlay_Basis layer (single layer, 3 materials for U/V/N) ---
    basis_layer_name = "Overlay_Basis"
    if basis_layer_name in gp_data.layers:
        bl = gp_data.layers[basis_layer_name]
        bl.clear()
    else:
        bl = gp_data.layers.new(basis_layer_name, set_active=False)
    if not bl.frames:
        basis_frame = bl.frames.new(0)
    else:
        basis_frame = bl.frames[0]
    
    basis_mats = {}
    for axis_key, color in _BASIS_COLORS.items():
        mat_name = f"CFTUV_Axis_{axis_key}"
        if mat_name in bpy.data.materials:
            mat = bpy.data.materials[mat_name]
        else:
            mat = bpy.data.materials.new(mat_name)
        if not mat.grease_pencil:
            bpy.data.materials.create_gpencil_data(mat)
        mat.grease_pencil.color = color[:4]
        mat.grease_pencil.show_fill = False
        # Ensure on gp_data
        mat_idx = None
        for i, slot in enumerate(gp_data.materials):
            if slot and slot.name == mat_name:
                mat_idx = i
                break
        if mat_idx is None:
            gp_data.materials.append(mat)
            mat_idx = len(gp_data.materials) - 1
        basis_mats[axis_key] = mat_idx
    
    # --- Loop Types layers (Loops_Chains, Loops_Boundary, Loops_Holes) ---
    loop_layer_names = ['Loops_Chains', 'Loops_Boundary', 'Loops_Holes']
    loop_layers = {}
    for ln in loop_layer_names:
        if ln in gp_data.layers:
            layer = gp_data.layers[ln]
            layer.clear()
        else:
            layer = gp_data.layers.new(ln, set_active=False)
        if not layer.frames:
            gp_frame = layer.frames.new(0)
        else:
            gp_frame = layer.frames[0]
        loop_layers[ln] = gp_frame
    
    # Pre-count elements for material generation
    chain_counter = 0
    loop_counter = 0
    hole_counter = 0
    
    def _get_element_mat(prefix, index, hue_offset, sat, val, gp_data):
        """Create/get per-element colored material for loop types."""
        hue = ((index * golden_ratio) + hue_offset) % 1.0
        r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
        mat_name = f"CFTUV_{prefix}{index:03d}"
        if mat_name in bpy.data.materials:
            mat = bpy.data.materials[mat_name]
        else:
            mat = bpy.data.materials.new(mat_name)
        if not mat.grease_pencil:
            bpy.data.materials.create_gpencil_data(mat)
        mat.grease_pencil.color = (r, g, b, 1.0)
        mat.grease_pencil.show_fill = False
        mat_idx = None
        for i, slot in enumerate(gp_data.materials):
            if slot and slot.name == mat_name:
                mat_idx = i
                break
        if mat_idx is None:
            gp_data.materials.append(mat)
            mat_idx = len(gp_data.materials) - 1
        return mat_idx
    
    # --- Patches layers per type (Patches_WALL, Patches_FLOOR, Patches_SLOPE) ---
    patch_types = {'WALL', 'FLOOR', 'SLOPE'}
    patch_layers = {}  # type_name → (gp_frame, dict of patch_idx → mat_idx)
    golden_ratio = 0.618033988749895
    
    for ptype in patch_types:
        layer_name = f"Patches_{ptype}"
        if layer_name in gp_data.layers:
            layer = gp_data.layers[layer_name]
            layer.clear()
        else:
            layer = gp_data.layers.new(layer_name, set_active=False)
        if not layer.frames:
            gp_frame = layer.frames.new(0)
        else:
            gp_frame = layer.frames[0]
        patch_layers[ptype] = gp_frame
    
    # Per-patch materials
    patch_mat_indices = []
    for pi, patch in enumerate(patch_results):
        hue = (pi * golden_ratio) % 1.0
        if patch['type'] == 'WALL':
            sat, val = 0.8, 0.9
        elif patch['type'] == 'SLOPE':
            sat, val = 0.7, 0.75
        else:
            sat, val = 0.5, 0.6
        r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
        
        mat_name = f"CFTUV_P{pi:03d}"
        if mat_name in bpy.data.materials:
            mat = bpy.data.materials[mat_name]
        else:
            mat = bpy.data.materials.new(mat_name)
        if not mat.grease_pencil:
            bpy.data.materials.create_gpencil_data(mat)
        mat.grease_pencil.color = (r, g, b, 1.0)
        mat.grease_pencil.show_fill = True
        mat.grease_pencil.fill_color = (r, g, b, 1.0)
        mat.grease_pencil.show_stroke = False
        
        mat_idx = None
        for i, slot in enumerate(gp_data.materials):
            if slot and slot.name == mat_name:
                mat_idx = i
                break
        if mat_idx is None:
            gp_data.materials.append(mat)
            mat_idx = len(gp_data.materials) - 1
        patch_mat_indices.append(mat_idx)
    
    # --- Draw per patch ---
    for pi, patch in enumerate(patch_results):
        c = patch['centroid']
        axis_len = 0.15
        
        # Overlay: Basis axes (all into single Overlay_Basis layer)
        _add_gp_stroke(basis_frame, [c, c + patch['seed_t'] * axis_len], basis_mats['U'], line_width=8)
        _add_gp_stroke(basis_frame, [c, c + patch['seed_b'] * axis_len], basis_mats['V'], line_width=8)
        _add_gp_stroke(basis_frame, [c, c + patch['normal'] * axis_len * 0.6], basis_mats['N'], line_width=6)
        
        # Patches fill — into type-specific layer
        ptype = patch['type']
        if ptype not in patch_layers:
            ptype = 'WALL'  # fallback
        p_frame = patch_layers[ptype]
        p_mat = patch_mat_indices[pi]
        m_verts = patch.get('mesh_verts', [])
        m_faces = patch.get('mesh_faces', [])
        for face_indices in m_faces:
            if len(face_indices) < 3:
                continue
            stroke = p_frame.strokes.new()
            stroke.material_index = p_mat
            stroke.line_width = 1
            stroke.use_cyclic = True
            stroke.points.add(len(face_indices))
            for i, vi in enumerate(face_indices):
                pt = m_verts[vi]
                stroke.points[i].co = (pt.x, pt.y, pt.z)
                stroke.points[i].strength = 1.0
                stroke.points[i].pressure = 1.0
        
        # Frame segments
        for lp in patch['loops']:
            for seg in lp.get('segments', []):
                vert_cos = seg.get('vert_cos', [])
                if len(vert_cos) < 2:
                    continue
                kind = seg.get('loop_kind', 'OUTER')
                role = seg.get('frame_role', 'FREE')
                
                if kind == 'HOLE':
                    style, width = 'Frame_HOLE', 3
                elif role == 'H_FRAME':
                    style, width = 'Frame_H', 6
                elif role == 'V_FRAME':
                    style, width = 'Frame_V', 6
                else:
                    style, width = 'Frame_FREE', 3
                
                f, m = frames_and_mats[style]
                _add_gp_stroke(f, vert_cos, m, line_width=width)
        
        # Loop Types: Chains (per-chain unique color)
        for chain in patch.get('all_chains', []):
            cos = chain.get('vert_cos', [])
            if len(cos) < 2:
                continue
            cm = _get_element_mat('Ch', chain_counter, 0.0, 0.9, 0.85, gp_data)
            chain_counter += 1
            pts = cos + [cos[0]] if chain.get('is_closed', False) else cos
            _add_gp_stroke(loop_layers['Loops_Chains'], pts, cm, line_width=4)
        
        # Loop Types: Boundary loops (per-loop unique color)
        for lp in patch['loops']:
            cos = lp.get('vert_cos', [])
            if len(cos) < 3:
                continue
            if lp.get('kind') == 'HOLE':
                # Holes → separate layer
                hm = _get_element_mat('Hl', hole_counter, 0.5, 0.6, 0.7, gp_data)
                hole_counter += 1
                _add_gp_stroke(loop_layers['Loops_Holes'], cos + [cos[0]], hm, line_width=5)
            else:
                lm = _get_element_mat('Lp', loop_counter, 0.25, 0.85, 0.9, gp_data)
                loop_counter += 1
                _add_gp_stroke(loop_layers['Loops_Boundary'], cos + [cos[0]], lm, line_width=5)
    
    # Apply visibility from settings
    if dbg_settings:
        _apply_layer_visibility(gp_data, dbg_settings)
    
    return gp_obj


def _apply_layer_visibility(gp_data, dbg_settings):
    """Синхронизирует visibility GP layers с toggle settings."""
    grp_patches = dbg_settings.get('grp_patches', True)
    grp_frame = dbg_settings.get('grp_frame', True)
    grp_loops = dbg_settings.get('grp_loops', True)
    grp_overlay = dbg_settings.get('grp_overlay', True)
    
    mapping = {
        'Patches_WALL':  grp_patches and dbg_settings.get('patches_wall', True),
        'Patches_FLOOR': grp_patches and dbg_settings.get('patches_floor', True),
        'Patches_SLOPE': grp_patches and dbg_settings.get('patches_slope', True),
        'Frame_H':       grp_frame and dbg_settings.get('frame_h', True),
        'Frame_V':       grp_frame and dbg_settings.get('frame_v', True),
        'Frame_FREE':    grp_frame and dbg_settings.get('frame_free', True),
        'Frame_HOLE':    grp_frame and dbg_settings.get('frame_hole', True),
        'Loops_Chains':  grp_loops,
        'Loops_Boundary': grp_loops,
        'Loops_Holes':   grp_loops,
        'Overlay_Basis': grp_overlay,
    }
    for layer_name, visible in mapping.items():
        if layer_name in gp_data.layers:
            gp_data.layers[layer_name].hide = not visible


def _enter_debug_mode(context, obj):
    """Входит в режим debug: анализ, создание GP."""
    s = context.scene.hotspotuv_settings
    was_object_mode = (obj.mode == 'OBJECT')
    
    if obj.mode != 'EDIT':
        bpy.ops.object.mode_set(mode='EDIT')
    
    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    
    # Object mode → все faces, Edit mode → только выделенные
    if was_object_mode:
        sel_faces = list(bm.faces)
    else:
        sel_faces = [f for f in bm.faces if f.select]
        if not sel_faces:
            bpy.ops.object.mode_set(mode='EDIT')
            return None  # Signal: nothing selected
    
    original_seams = [e.seam for e in bm.edges]
    for e in bm.edges:
        if not e.smooth:
            e.seam = True
    
    patch_results = analyze_all_patches(bm, sel_faces, obj)
    
    for i, e in enumerate(bm.edges):
        e.seam = original_seams[i]
    bmesh.update_edit_mesh(obj.data)
    
    bpy.ops.object.mode_set(mode='OBJECT')
    
    dbg_settings = {
        'grp_patches': s.dbg_grp_patches,
        'grp_frame': s.dbg_grp_frame,
        'grp_loops': s.dbg_grp_loops,
        'grp_overlay': s.dbg_grp_overlay,
        'patches_wall': s.dbg_patches_wall,
        'patches_floor': s.dbg_patches_floor,
        'patches_slope': s.dbg_patches_slope,
        'frame_h': s.dbg_frame_h,
        'frame_v': s.dbg_frame_v,
        'frame_free': s.dbg_frame_free,
        'frame_hole': s.dbg_frame_hole,
    }
    
    gp_obj = create_debug_visualization(patch_results, obj, dbg_settings)
    
    # Скрываем source mesh, выделяем GP debug object
    obj.hide_viewport = True
    bpy.ops.object.select_all(action='DESELECT')
    gp_obj.select_set(True)
    context.view_layer.objects.active = gp_obj
    
    # Track state
    s.dbg_active = True
    s.dbg_source_object = obj.name
    
    # Console log
    print("=" * 60)
    print("CFTUV Debug Analysis")
    print("=" * 60)
    for pi, p in enumerate(patch_results):
        n_faces = len(p['faces'])
        roles = [si['frame_role'] for si in p['all_segments']]
        chains_info = [f"nb={c['neighbor']}" for c in p.get('all_chains', [])]
        print(f"  Patch {pi}: {p['type']} | {n_faces}f | "
              f"loops:{len(p['loops'])} chains:{len(p.get('all_chains',[]))} | "
              f"segs:[{' '.join(roles)}] | "
              f"chains:[{' '.join(chains_info)}]")
    print("=" * 60)
    
    total_patches = len(patch_results)
    walls = sum(1 for p in patch_results if p['type'] == 'WALL')
    floors = sum(1 for p in patch_results if p['type'] == 'FLOOR')
    slopes = sum(1 for p in patch_results if p['type'] == 'SLOPE')
    singles = sum(1 for p in patch_results if len(p['faces']) == 1)
    total_h = sum(1 for p in patch_results for si in p['all_segments'] if si['frame_role'] == 'H_FRAME')
    total_v = sum(1 for p in patch_results for si in p['all_segments'] if si['frame_role'] == 'V_FRAME')
    total_free = sum(1 for p in patch_results for si in p['all_segments'] if si['frame_role'] == 'FREE')
    total_holes = sum(1 for p in patch_results for lp in p['loops'] if lp['kind'] == 'HOLE')
    total_segs = sum(len(p['all_segments']) for p in patch_results)
    total_chains = sum(len(p.get('all_chains', [])) for p in patch_results)
    total_loops = sum(len(p['loops']) for p in patch_results)
    
    return (f"Patches: {total_patches} (W:{walls} F:{floors} S:{slopes} 1f:{singles}) | "
            f"Loops: {total_loops} Chains: {total_chains} Holes: {total_holes} | "
            f"Seg: {total_segs} H:{total_h} V:{total_v} Free:{total_free}")


def _exit_debug_mode(context):
    """Выходит из режима debug: удаляет GP."""
    s = context.scene.hotspotuv_settings
    source_name = s.dbg_source_object
    
    # Ensure OBJECT mode
    if context.active_object and context.active_object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    
    if source_name and source_name in bpy.data.objects:
        source_obj = bpy.data.objects[source_name]
        _clear_gp_debug(source_obj)
        source_obj.hide_viewport = False
        bpy.ops.object.select_all(action='DESELECT')
        source_obj.select_set(True)
        context.view_layer.objects.active = source_obj
    
    s.dbg_active = False
    s.dbg_source_object = ""


class HOTSPOTUV_OT_DebugAnalysis(bpy.types.Operator):
    bl_idname = "hotspotuv.debug_analysis"
    bl_label = "Debug: Toggle Analysis"
    bl_description = "Toggle debug analysis mode ON/OFF"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        s = context.scene.hotspotuv_settings
        if s.dbg_active:
            return True  # Always allow turning off
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        s = context.scene.hotspotuv_settings
        
        if s.dbg_active:
            # OFF: exit debug mode
            _exit_debug_mode(context)
            self.report({"INFO"}, "Debug analysis OFF")
        else:
            # ON: enter debug mode
            obj = context.active_object
            if not obj or obj.type != 'MESH':
                self.report({"WARNING"}, "Select a mesh object")
                return {"CANCELLED"}
            report = _enter_debug_mode(context, obj)
            if report is None:
                self.report({"WARNING"}, "No faces selected in Edit Mode")
                return {"CANCELLED"}
            self.report({"INFO"}, report)
        
        return {"FINISHED"}


class HOTSPOTUV_OT_DebugClear(bpy.types.Operator):
    bl_idname = "hotspotuv.debug_clear"
    bl_label = "Debug: Force Clear"
    bl_description = "Force remove all debug visualization"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        s = context.scene.hotspotuv_settings
        
        if context.active_object and context.active_object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        # Clear for tracked source
        source_name = s.dbg_source_object
        if source_name and source_name in bpy.data.objects:
            source_obj = bpy.data.objects[source_name]
            _clear_gp_debug(source_obj)
            source_obj.hide_viewport = False
        
        # Also try current active object
        obj = context.active_object
        if obj:
            if obj.name.startswith(GP_DEBUG_PREFIX):
                src_name = obj.name[len(GP_DEBUG_PREFIX):]
                src = bpy.data.objects.get(src_name)
                if src:
                    _clear_gp_debug(src)
                    src.hide_viewport = False
            elif obj.type == 'MESH':
                _clear_gp_debug(obj)
        
        s.dbg_active = False
        s.dbg_source_object = ""
        self.report({"INFO"}, "Debug cleared")
        return {"FINISHED"}


class HOTSPOTUV_OT_DebugToggleLayer(bpy.types.Operator):
    """Toggle visibility of a GP debug layer directly from panel."""
    bl_idname = "hotspotuv.debug_toggle_layer"
    bl_label = "Toggle Layer"
    bl_options = {"REGISTER", "UNDO"}
    
    layer_name: StringProperty()
    
    def execute(self, context):
        s = context.scene.hotspotuv_settings
        source_name = s.dbg_source_object
        if not source_name:
            return {"CANCELLED"}
        
        gp_name = GP_DEBUG_PREFIX + source_name
        if gp_name not in bpy.data.objects:
            return {"CANCELLED"}
        
        gp_obj = bpy.data.objects[gp_name]
        if gp_obj.type != 'GPENCIL':
            return {"CANCELLED"}
        
        gp_data = gp_obj.data
        if self.layer_name in gp_data.layers:
            layer = gp_data.layers[self.layer_name]
            layer.hide = not layer.hide
        
        return {"FINISHED"}


class HOTSPOTUV_OT_DebugToggleGroup(bpy.types.Operator):
    """Toggle visibility of a debug layer group (Patches/Frame/Overlay)."""
    bl_idname = "hotspotuv.debug_toggle_group"
    bl_label = "Toggle Group"
    bl_options = {"REGISTER", "UNDO"}
    
    group_name: StringProperty()  # 'patches', 'frame', 'overlay'
    
    # Group → GP layer name prefixes
    _GROUP_LAYERS = {
        'patches': ['Patches_WALL', 'Patches_FLOOR', 'Patches_SLOPE'],
        'frame': ['Frame_H', 'Frame_V', 'Frame_FREE', 'Frame_HOLE'],
        'loops': ['Loops_Chains', 'Loops_Boundary', 'Loops_Holes'],
        'overlay': ['Overlay_Basis'],
    }
    
    def execute(self, context):
        s = context.scene.hotspotuv_settings
        source_name = s.dbg_source_object
        if not source_name:
            return {"CANCELLED"}
        
        # Toggle the group setting
        prop_name = f"dbg_grp_{self.group_name}"
        new_val = not getattr(s, prop_name, True)
        setattr(s, prop_name, new_val)
        
        # Sync GP layer visibility
        gp_name = GP_DEBUG_PREFIX + source_name
        if gp_name not in bpy.data.objects:
            return {"FINISHED"}
        gp_obj = bpy.data.objects[gp_name]
        if gp_obj.type != 'GPENCIL':
            return {"FINISHED"}
        gp_data = gp_obj.data
        
        layer_names = self._GROUP_LAYERS.get(self.group_name, [])
        for ln in layer_names:
            if ln in gp_data.layers:
                gp_data.layers[ln].hide = not new_val
        
        return {"FINISHED"}

# ============================================================
# HYBRID ALIGNMENT LOGIC
# ============================================================

def orient_scale_and_position_island(uv_layer, island):
    sorted_faces = sorted(
        island.faces, 
        key=lambda f: f.calc_area() * (max(0, f.normal.dot(island.avg_normal)) ** 4), 
        reverse=True
    )
    if not sorted_faces: return
    seed_face = sorted_faces[0]

    island_up = find_island_up(island)
    seed_t, seed_b = calc_surface_basis(seed_face.normal, island_up)
    
    ideal_uvs = {}
    for l in seed_face.loops:
        u = l.vert.co.dot(seed_t) * FINAL_UV_SCALE
        v = l.vert.co.dot(seed_b) * FINAL_UV_SCALE
        ideal_uvs[l.vert.index] = Vector((u, v))
        
    current_uvs = {}
    for l in seed_face.loops:
        current_uvs[l.vert.index] = l[uv_layer].uv.copy()
        
    longest_edge = max(seed_face.edges, key=lambda e: e.calc_length())
    v1, v2 = longest_edge.verts[0].index, longest_edge.verts[1].index
    
    tgt_u1, tgt_u2 = ideal_uvs[v1], ideal_uvs[v2]
    src_u1, src_u2 = current_uvs[v1], current_uvs[v2]
    
    tgt_vec = tgt_u2 - tgt_u1
    src_vec = src_u2 - src_u1
    
    if src_vec.length_squared < 1e-6: return
    
    delta_angle = math.atan2(tgt_vec.y, tgt_vec.x) - math.atan2(src_vec.y, src_vec.x)
    cos_a, sin_a = math.cos(delta_angle), math.sin(delta_angle)
    
    scale = tgt_vec.length / src_vec.length if src_vec.length > 1e-6 else 1.0
    
    for f in island.faces:
        for l in f.loops:
            p = l[uv_layer].uv - src_u1
            rx = (p.x * cos_a - p.y * sin_a) * scale
            ry = (p.x * sin_a + p.y * cos_a) * scale
            l[uv_layer].uv = Vector((rx, ry)) + tgt_u1

# ============================================================
# VALIDATION HELPERS
# ============================================================

def validate_edit_mesh(context, require_selection=True, selection_type='FACE'):
    """
    Валидация контекста для операторов.
    Возвращает (success, error_message, bm)
    """
    obj = context.object
    if obj is None:
        return False, "No active object", None
    if obj.type != 'MESH':
        return False, "Active object is not a mesh", None
    if obj.mode != 'EDIT':
        return False, "Must be in Edit Mode", None
    
    mesh = obj.data
    if len(mesh.vertices) == 0:
        return False, "Mesh has no vertices", None
    
    bm = bmesh.from_edit_mesh(mesh)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    
    if require_selection:
        if selection_type == 'FACE':
            if not any(f.select for f in bm.faces):
                return False, "No faces selected", bm
        elif selection_type == 'EDGE':
            if not any(e.select for e in bm.edges):
                return False, "No edges selected", bm
    
    return True, "", bm

# ============================================================
# DOCKING & WELDING
# ============================================================

def compute_best_fit_transform(anchor_uvs_list, target_uvs_list):
    """
    Вычисляет жесткую трансформацию (только поворот и смещение).
    anchor = Static Island (Цель, куда стыкуемся)
    target = Moving Island (Остров, который мы двигаем)
    """
    if not anchor_uvs_list or len(anchor_uvs_list) != len(target_uvs_list):
        return 0.0, Vector((0.0, 0.0)), Vector((0.0, 0.0))
    
    n = len(anchor_uvs_list)
    anchor_centroid = sum(anchor_uvs_list, Vector((0.0, 0.0))) / n
    target_centroid = sum(target_uvs_list, Vector((0.0, 0.0))) / n
    
    # Правильный порядок вычисления угла от Target к Anchor
    num = 0.0
    den = 0.0
    for i in range(n):
        s = target_uvs_list[i] - target_centroid
        t = anchor_uvs_list[i] - anchor_centroid
        num += s.x * t.y - s.y * t.x
        den += s.x * t.x + s.y * t.y
    
    angle = math.atan2(num, den) if abs(num) > 1e-10 or abs(den) > 1e-10 else 0.0
    return angle, anchor_centroid, target_centroid

def dock_island_to_anchor(uv_layer, target_island_faces, anchor_uvs_list, target_uvs_list, 
                          target_vert_indices=None, fit_vertices=False, unwrap_interior=False):
    """
    Применяет вычисленную трансформацию и (опционально) сваривает и пинит вершины шва.
    """
    if not anchor_uvs_list or len(anchor_uvs_list) != len(target_uvs_list):
        return False
    
    angle, anchor_centroid, target_centroid = compute_best_fit_transform(
        anchor_uvs_list, target_uvs_list
    )
    
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    
    # 1. Жесткая трансформация (Rigid Body Transform) всего лоскута
    for f in target_island_faces:
        for l in f.loops:
            p = l[uv_layer].uv - target_centroid
            rx = p.x * cos_a - p.y * sin_a
            ry = p.x * sin_a + p.y * cos_a
            l[uv_layer].uv = Vector((rx, ry)) + anchor_centroid
    
    # 2. Точечный Snap и Pinning (если включено)
    if fit_vertices and target_vert_indices is not None:
        vert_to_anchor_uv = {target_vert_indices[i]: anchor_uvs_list[i] for i in range(len(target_vert_indices))}
        
        for f in target_island_faces:
            for l in f.loops:
                if l.vert.index in vert_to_anchor_uv:
                    l[uv_layer].uv = vert_to_anchor_uv[l.vert.index].copy()
                    
                    # ПРИКАЛЫВАЕМ точки шва булавкой для дальнейшего Unwrap
                    if unwrap_interior:
                        l[uv_layer].pin_uv = True
    
    return True

def get_edge_uv_coords(edge, face, uv_layer):
    """Получает UV координаты ребра в контексте данного face."""
    uv1 = uv2 = None
    v1_idx, v2_idx = edge.verts[0].index, edge.verts[1].index
    
    for l in face.loops:
        if l.vert.index == v1_idx:
            uv1 = l[uv_layer].uv.copy()
        elif l.vert.index == v2_idx:
            uv2 = l[uv_layer].uv.copy()
    
    return (uv1, uv2, v1_idx, v2_idx) if uv1 is not None and uv2 is not None else None

def get_geometry_island_for_face(face, bm):
    """
    Находит геометрический лоскут (island) для данного face.
    Использует get_expanded_islands с одним face как seed.
    """
    islands_data = get_expanded_islands(bm, [face])
    if islands_data:
        return islands_data[0]['full']
    return [face]

def build_island_graph(selected_edges, bm):
    """
    Строит граф связей между геометрическими лоскутами.
    Собирает ВСЕ общие рёбра между каждой парой островов.
    
    Возвращает:
    - islands: {island_id: {'faces': [BMFace], 'area': float, 'id': int}}
    - graph: {island_id: {neighbor_id: [(edge, my_face, neighbor_face), ...], ...}}
    - face_to_island: {face.index: island_id}
    """
    face_to_island = {}
    islands = {}
    island_counter = 0
    
    # 1. Собираем все уникальные лоскуты из выделенных рёбер
    for edge in selected_edges:
        if len(edge.link_faces) != 2:
            continue
        
        for face in edge.link_faces:
            if face.index in face_to_island:
                continue
            
            # Находим весь лоскут для этого face
            island_faces = get_geometry_island_for_face(face, bm)
            
            # Проверяем, может этот лоскут уже зарегистрирован через другой face
            existing_island = None
            for f in island_faces:
                if f.index in face_to_island:
                    existing_island = face_to_island[f.index]
                    break
            
            if existing_island is not None:
                # Регистрируем все faces этого лоскута
                for f in island_faces:
                    face_to_island[f.index] = existing_island
            else:
                # Новый лоскут
                island_id = island_counter
                island_counter += 1
                
                # Вычисляем 3D площадь
                isl_info = IslandInfo(island_faces, island_id)
                analyze_island_properties(isl_info)
                
                islands[island_id] = {
                    'faces': island_faces,
                    'area': isl_info.area,
                    'id': island_id
                }
                
                for f in island_faces:
                    face_to_island[f.index] = island_id
    
    # 2. Строим граф связей - собираем ВСЕ общие рёбра между парами
    # graph[island_id] = {neighbor_id: [(edge, my_face, neighbor_face), ...], ...}
    graph = {isl_id: {} for isl_id in islands}
    processed_edges = set()
    
    for edge in selected_edges:
        if len(edge.link_faces) != 2:
            continue
        if edge.index in processed_edges:
            continue
        processed_edges.add(edge.index)
        
        face_a, face_b = edge.link_faces[0], edge.link_faces[1]
        
        island_a_id = face_to_island.get(face_a.index)
        island_b_id = face_to_island.get(face_b.index)
        
        if island_a_id is None or island_b_id is None:
            continue
        if island_a_id == island_b_id:
            continue
        
        # Добавляем ребро в список общих рёбер для этой пары
        if island_b_id not in graph[island_a_id]:
            graph[island_a_id][island_b_id] = []
        graph[island_a_id][island_b_id].append((edge, face_a, face_b))
        
        if island_a_id not in graph[island_b_id]:
            graph[island_b_id][island_a_id] = []
        graph[island_b_id][island_a_id].append((edge, face_b, face_a))
    
    return islands, graph, face_to_island

def find_root_island(islands, direction):
    """
    Находит корневой остров для начала BFS.
    AUTO → max area, REVERSE → min area
    """
    if not islands:
        return None
    
    if direction == 'AUTO':
        return max(islands.keys(), key=lambda x: islands[x]['area'])
    else:  # REVERSE
        return min(islands.keys(), key=lambda x: islands[x]['area'])

def find_connected_components(islands, graph):
    """
    Находит все связные компоненты в графе островов.
    Каждая компонента — независимая цепочка лоскутов.
    
    Возвращает: [set(island_ids), set(island_ids), ...]
    """
    visited = set()
    components = []
    
    for island_id in islands:
        if island_id in visited:
            continue
        
        # BFS для поиска компоненты
        component = set()
        queue = [island_id]
        
        while queue:
            curr = queue.pop(0)
            if curr in visited:
                continue
            visited.add(curr)
            component.add(curr)
            
            # Добавляем всех соседей
            for neighbor_id in graph[curr].keys():
                if neighbor_id not in visited:
                    queue.append(neighbor_id)
        
        components.append(component)
    
    return components

def dock_all_chains(islands, graph, bm, context, direction, fit_vertices, unwrap_interior):
    """
    Обрабатывает ВСЕ независимые цепочки.
    Каждая цепочка получает свой корень по direction.
    
    Возвращает: (total_docked_count, updated_bm)
    """
    components = find_connected_components(islands, graph)
    
    total_docked = 0
    
    for component in components:
        if len(component) < 2:
            # Одиночный остров — нечего стыковать
            continue
        
        # Находим корень ВНУТРИ этой компоненты
        if direction == 'AUTO':
            root_id = max(component, key=lambda x: islands[x]['area'])
        else:  # REVERSE
            root_id = min(component, key=lambda x: islands[x]['area'])
        
        # Послойный BFS с unwrap после каждого уровня
        docked_count, bm = dock_chain_bfs_layered(
            root_id, islands, graph, bm, context, fit_vertices, unwrap_interior
        )
        
        total_docked += docked_count
    
    return total_docked, bm

def dock_chain_bfs_layered(root_id, islands, graph, bm, context, fit_vertices, unwrap_interior):
    """
    Послойный BFS с unwrap после каждого уровня.
    
    После стыковки каждого уровня делается unwrap, чтобы следующий уровень
    использовал АКТУАЛЬНЫЕ UV координаты.
    
    Возвращает: количество успешных стыковок
    """
    uv_layer = bm.loops.layers.uv.verify()
    
    docked_count = 0
    visited = {root_id}
    current_level = [root_id]
    
    while True:
        next_level = []
        level_docked_face_indices = []  # Храним индексы, т.к. BMesh будет пересоздан
        
        for anchor_id in current_level:
            for neighbor_id, edges_data in graph[anchor_id].items():
                if neighbor_id in visited:
                    continue
                
                # Собираем UV координаты (АКТУАЛЬНЫЕ после предыдущего unwrap)
                anchor_uvs_list = []
                target_uvs_list = []
                target_vert_indices = []
                processed_verts = set()
                
                for (edge, anchor_face, neighbor_face) in edges_data:
                    for vert in edge.verts:
                        if vert.index in processed_verts:
                            continue
                        processed_verts.add(vert.index)
                        
                        anchor_uv = None
                        for l in anchor_face.loops:
                            if l.vert.index == vert.index:
                                anchor_uv = l[uv_layer].uv.copy()
                                break
                        
                        target_uv = None
                        for l in neighbor_face.loops:
                            if l.vert.index == vert.index:
                                target_uv = l[uv_layer].uv.copy()
                                break
                        
                        if anchor_uv is not None and target_uv is not None:
                            anchor_uvs_list.append(anchor_uv)
                            target_uvs_list.append(target_uv)
                            target_vert_indices.append(vert.index)
                
                if len(anchor_uvs_list) < 2:
                    visited.add(neighbor_id)
                    next_level.append(neighbor_id)
                    continue
                
                target_faces = islands[neighbor_id]['faces']
                
                success = dock_island_to_anchor(
                    uv_layer, target_faces, anchor_uvs_list, target_uvs_list,
                    target_vert_indices=target_vert_indices,
                    fit_vertices=fit_vertices,
                    unwrap_interior=unwrap_interior
                )
                
                if success:
                    docked_count += 1
                    # Сохраняем индексы faces для unwrap
                    level_docked_face_indices.extend([f.index for f in target_faces])
                
                visited.add(neighbor_id)
                next_level.append(neighbor_id)
        
        if not next_level:
            break
        
        # Unwrap текущего уровня ПЕРЕД переходом к следующему
        if fit_vertices and unwrap_interior and level_docked_face_indices:
            # Сохраняем выделение рёбер
            orig_edge_sel = [e.index for e in bm.edges if e.select]
            
            # Выделяем faces текущего уровня
            for f in bm.faces:
                f.select = f.index in level_docked_face_indices
            
            bmesh.update_edit_mesh(context.edit_object.data)
            
            # Conformal unwrap (не тронет pinned вершины шва)
            bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)
            
            # Пересоздаём BMesh для актуальных UV
            bm.free()
            bm = bmesh.from_edit_mesh(context.edit_object.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()
            
            # Очищаем pins
            for f_idx in level_docked_face_indices:
                if f_idx < len(bm.faces):
                    for l in bm.faces[f_idx].loops:
                        l[uv_layer].pin_uv = False
            
            # Обновляем ссылки на faces в islands (они теперь из нового BMesh)
            for isl_id in islands:
                new_faces = [bm.faces[f.index] for f in islands[isl_id]['faces'] if f.index < len(bm.faces)]
                islands[isl_id]['faces'] = new_faces
            
            # Обновляем ссылки в graph
            for isl_id in graph:
                for neighbor_id in graph[isl_id]:
                    new_edges_data = []
                    for (edge, anchor_face, neighbor_face) in graph[isl_id][neighbor_id]:
                        if edge.index < len(bm.edges) and anchor_face.index < len(bm.faces) and neighbor_face.index < len(bm.faces):
                            new_edges_data.append((
                                bm.edges[edge.index],
                                bm.faces[anchor_face.index],
                                bm.faces[neighbor_face.index]
                            ))
                    graph[isl_id][neighbor_id] = new_edges_data
            
            # Восстанавливаем выделение рёбер
            for f in bm.faces:
                f.select = False
            for e_idx in orig_edge_sel:
                if e_idx < len(bm.edges):
                    bm.edges[e_idx].select = True
        
        current_level = next_level
    
    return docked_count, bm  # Возвращаем обновлённый bm

def weld_island_uvs(uv_layer, island, distance=0.001):
    """
    Сшивает UV вершины ВНУТРИ одного острова.
    
    ВАЖНО: Эта функция работает ТОЛЬКО с faces одного острова.
    UV вершины разных островов НЕ затрагиваются и НЕ мёржатся.
    """
    vert_to_loops = {}
    for f in island.faces:
        for l in f.loops:
            vert_to_loops.setdefault(l.vert.index, []).append(l)
            
    for v_idx, loops in vert_to_loops.items():
        clusters = []
        for l in loops:
            uv = l[uv_layer].uv
            placed = False
            for cluster in clusters:
                if (cluster['uv'] - uv).length < distance:
                    cluster['loops'].append(l)
                    cluster['uv'] = sum((loop[uv_layer].uv for loop in cluster['loops']), Vector((0.0, 0.0))) / len(cluster['loops'])
                    placed = True
                    break
            if not placed:
                clusters.append({'uv': uv.copy(), 'loops': [l]})
                
        for cluster in clusters:
            exact_uv = cluster['uv']
            for l in cluster['loops']:
                l[uv_layer].uv = exact_uv.copy()

def align_connected_islands(islands_list, links, uv_layer):
    """
    Hybrid alignment: 
    - Угол (rotation) из direction САМОГО ДЛИННОГО shared edge — детерминированный,
      не даёт ложных 90/180/270° поворотов.
    - Позиция (translation) из centroid ВСЕХ shared вершин — точнее чем 2 точки.
    """
    links.sort(key=lambda x: x['shared_length'], reverse=True)
    parent = {isl.index: isl.index for isl in islands_list}
    cluster_elements = {isl.index: [isl.index] for isl in islands_list}
    
    def find(i):
        if parent[i] == i: return i
        parent[i] = find(parent[i])
        return parent[i]

    for link in links:
        idx_A, idx_B = link['isl_a'], link['isl_b']
        isl_A, isl_B = islands_list[idx_A], islands_list[idx_B]
        
        if isl_A.type != isl_B.type: continue
        
        root_A, root_B = find(idx_A), find(idx_B)
        if root_A == root_B: continue 
            
        shared_len = link['shared_length']
        max_perim, min_perim = max(isl_A.perimeter, isl_B.perimeter), min(isl_A.perimeter, isl_B.perimeter)
        is_valid_contact = False
        
        if max_perim > 1e-5 and (shared_len / max_perim) >= 0.02:
            is_valid_contact = True
        elif min_perim > 1e-5 and (shared_len / min_perim) >= 0.30:
            area_root_A = sum(islands_list[i].area for i in cluster_elements[root_A])
            area_root_B = sum(islands_list[i].area for i in cluster_elements[root_B])
            min_cluster_area = min(area_root_A, area_root_B)
            min_isl_area = min(isl_A.area, isl_B.area)
            if min_cluster_area <= (min_isl_area * 5.0 + 1e-4):
                is_valid_contact = True

        if not is_valid_contact: continue
                
        area_A = sum(islands_list[i].area for i in cluster_elements[root_A])
        area_B = sum(islands_list[i].area for i in cluster_elements[root_B])
        
        if area_B > area_A:
            idx_A, idx_B = idx_B, idx_A
            isl_A, isl_B = isl_B, isl_A
            root_A, root_B = root_B, root_A

        # ── ROTATION: по direction самого длинного shared edge ──
        v1_id, v2_id = link['longest_edge_verts']
        tgt_v1 = tgt_v2 = src_v1 = src_v2 = None
        
        for f in isl_A.faces:
            for l in f.loops:
                if l.vert.index == v1_id: tgt_v1 = l[uv_layer].uv.copy()
                if l.vert.index == v2_id: tgt_v2 = l[uv_layer].uv.copy()
            if tgt_v1 and tgt_v2: break
                    
        for f in isl_B.faces:
            for l in f.loops:
                if l.vert.index == v1_id: src_v1 = l[uv_layer].uv.copy()
                if l.vert.index == v2_id: src_v2 = l[uv_layer].uv.copy()
            if src_v1 and src_v2: break
                    
        if tgt_v1 is None or tgt_v2 is None or src_v1 is None or src_v2 is None: continue
        
        tgt_vec = tgt_v2 - tgt_v1
        src_vec = src_v2 - src_v1
        if tgt_vec.length_squared < 1e-6 or src_vec.length_squared < 1e-6: continue
        
        delta_angle = math.atan2(tgt_vec.y, tgt_vec.x) - math.atan2(src_vec.y, src_vec.x)
        cos_a, sin_a = math.cos(delta_angle), math.sin(delta_angle)
        
        # ── TRANSLATION: centroid всех shared вершин ──
        shared_vert_ids = set(link['shared_verts'])
        
        anchor_uvs = {}
        for f in isl_A.faces:
            for l in f.loops:
                if l.vert.index in shared_vert_ids and l.vert.index not in anchor_uvs:
                    anchor_uvs[l.vert.index] = l[uv_layer].uv.copy()
        
        source_uvs = {}
        for f in isl_B.faces:
            for l in f.loops:
                if l.vert.index in shared_vert_ids and l.vert.index not in source_uvs:
                    source_uvs[l.vert.index] = l[uv_layer].uv.copy()
        
        common_verts = set(anchor_uvs.keys()) & set(source_uvs.keys())
        if not common_verts: continue
        
        anchor_centroid = sum(anchor_uvs.values(), Vector((0.0, 0.0))) / len(anchor_uvs)
        source_centroid = sum(source_uvs.values(), Vector((0.0, 0.0))) / len(source_uvs)
        
        # ── APPLY: rotate around source_centroid, then translate to anchor_centroid ──
        for child_idx in cluster_elements[root_B]:
            child_isl = islands_list[child_idx]
            for f in child_isl.faces:
                for l in f.loops:
                    p = l[uv_layer].uv - source_centroid
                    l[uv_layer].uv = Vector((p.x * cos_a - p.y * sin_a, p.x * sin_a + p.y * cos_a)) + anchor_centroid
                    
        parent[root_B] = root_A
        cluster_elements[root_A].extend(cluster_elements[root_B])
        cluster_elements[root_B] = []



def _collect_shared_uv_correspondences(anchor_island, target_island, shared_vert_ids, uv_layer):
    anchor_uvs = {}
    for f in anchor_island.faces:
        for l in f.loops:
            if l.vert.index in shared_vert_ids and l.vert.index not in anchor_uvs:
                anchor_uvs[l.vert.index] = l[uv_layer].uv.copy()

    target_uvs = {}
    for f in target_island.faces:
        for l in f.loops:
            if l.vert.index in shared_vert_ids and l.vert.index not in target_uvs:
                target_uvs[l.vert.index] = l[uv_layer].uv.copy()

    common_verts = sorted(set(anchor_uvs.keys()) & set(target_uvs.keys()))
    return [anchor_uvs[v_id] for v_id in common_verts], [target_uvs[v_id] for v_id in common_verts]

def _is_valid_island_contact(link, isl_A, isl_B, cluster_elements, islands_list, root_A, root_B):
    if isl_A.type != isl_B.type:
        return False

    shared_len = link['shared_length']
    max_perim = max(isl_A.perimeter, isl_B.perimeter)
    min_perim = min(isl_A.perimeter, isl_B.perimeter)

    if max_perim > 1e-5 and (shared_len / max_perim) >= 0.02:
        return True

    if min_perim > 1e-5 and (shared_len / min_perim) >= 0.30:
        area_root_A = sum(islands_list[i].area for i in cluster_elements[root_A])
        area_root_B = sum(islands_list[i].area for i in cluster_elements[root_B])
        min_cluster_area = min(area_root_A, area_root_B)
        min_isl_area = min(isl_A.area, isl_B.area)
        return min_cluster_area <= (min_isl_area * 5.0 + 1e-4)

    return False

def _collect_cluster_frontier_links(root_A, root_B, cluster_elements, links):
    cluster_A = set(cluster_elements[root_A])
    cluster_B = set(cluster_elements[root_B])
    frontier_links = []

    for frontier_link in links:
        idx_A = frontier_link['isl_a']
        idx_B = frontier_link['isl_b']

        if idx_A in cluster_A and idx_B in cluster_B:
            frontier_links.append((idx_A, idx_B, frontier_link))
        elif idx_B in cluster_A and idx_A in cluster_B:
            frontier_links.append((idx_B, idx_A, frontier_link))

    return frontier_links

def _collect_cluster_frontier_correspondences(frontier_links, islands_list, uv_layer):
    anchor_uvs_list = []
    target_uvs_list = []

    for anchor_idx, target_idx, frontier_link in frontier_links:
        link_anchor_uvs, link_target_uvs = _collect_shared_uv_correspondences(
            islands_list[anchor_idx],
            islands_list[target_idx],
            set(frontier_link['shared_verts']),
            uv_layer
        )
        if not link_anchor_uvs:
            continue

        anchor_uvs_list.extend(link_anchor_uvs)
        target_uvs_list.extend(link_target_uvs)

    return anchor_uvs_list, target_uvs_list

def _compute_link_rotation_delta(anchor_island, target_island, frontier_link, uv_layer):
    v1_id, v2_id = frontier_link['longest_edge_verts']
    tgt_v1 = tgt_v2 = src_v1 = src_v2 = None

    for f in anchor_island.faces:
        for l in f.loops:
            if l.vert.index == v1_id:
                tgt_v1 = l[uv_layer].uv.copy()
            elif l.vert.index == v2_id:
                tgt_v2 = l[uv_layer].uv.copy()
        if tgt_v1 is not None and tgt_v2 is not None:
            break

    for f in target_island.faces:
        for l in f.loops:
            if l.vert.index == v1_id:
                src_v1 = l[uv_layer].uv.copy()
            elif l.vert.index == v2_id:
                src_v2 = l[uv_layer].uv.copy()
        if src_v1 is not None and src_v2 is not None:
            break

    if tgt_v1 is None or tgt_v2 is None or src_v1 is None or src_v2 is None:
        return None

    tgt_vec = tgt_v2 - tgt_v1
    src_vec = src_v2 - src_v1
    if tgt_vec.length_squared < 1e-6 or src_vec.length_squared < 1e-6:
        return None

    return math.atan2(tgt_vec.y, tgt_vec.x) - math.atan2(src_vec.y, src_vec.x)

def _compute_weighted_frontier_rotation(frontier_links, islands_list, uv_layer):
    sin_sum = 0.0
    cos_sum = 0.0
    total_weight = 0.0

    for anchor_idx, target_idx, frontier_link in frontier_links:
        delta = _compute_link_rotation_delta(
            islands_list[anchor_idx],
            islands_list[target_idx],
            frontier_link,
            uv_layer
        )
        if delta is None:
            continue

        weight = max(frontier_link['shared_length'], 1e-6)
        sin_sum += math.sin(delta) * weight
        cos_sum += math.cos(delta) * weight
        total_weight += weight

    if total_weight <= 0.0:
        return None

    return math.atan2(sin_sum, cos_sum)

def align_connected_islands(islands_list, links, uv_layer):
    """
    Cluster-aware final docking.
    Rotation is stabilized by a weighted average of deterministic seam directions,
    while translation still uses all frontier seam correspondences.
    This reduces drift/yaw without the 180-degree flips seen in the pure best-fit solve.
    """
    links.sort(key=lambda x: x['shared_length'], reverse=True)
    parent = {isl.index: isl.index for isl in islands_list}
    cluster_elements = {isl.index: [isl.index] for isl in islands_list}

    def find(i):
        if parent[i] == i:
            return i
        parent[i] = find(parent[i])
        return parent[i]

    for link in links:
        idx_A, idx_B = link['isl_a'], link['isl_b']
        isl_A, isl_B = islands_list[idx_A], islands_list[idx_B]

        root_A, root_B = find(idx_A), find(idx_B)
        if root_A == root_B:
            continue
        if not _is_valid_island_contact(link, isl_A, isl_B, cluster_elements, islands_list, root_A, root_B):
            continue

        area_A = sum(islands_list[i].area for i in cluster_elements[root_A])
        area_B = sum(islands_list[i].area for i in cluster_elements[root_B])
        if area_B > area_A:
            root_A, root_B = root_B, root_A

        frontier_links = _collect_cluster_frontier_links(root_A, root_B, cluster_elements, links)
        if not frontier_links:
            continue

        delta_angle = _compute_weighted_frontier_rotation(frontier_links, islands_list, uv_layer)
        if delta_angle is None:
            continue

        anchor_uvs_list, target_uvs_list = _collect_cluster_frontier_correspondences(
            frontier_links, islands_list, uv_layer
        )
        if not anchor_uvs_list or len(anchor_uvs_list) != len(target_uvs_list):
            continue

        n = len(anchor_uvs_list)
        anchor_centroid = sum(anchor_uvs_list, Vector((0.0, 0.0))) / n
        target_centroid = sum(target_uvs_list, Vector((0.0, 0.0))) / n
        cos_a, sin_a = math.cos(delta_angle), math.sin(delta_angle)

        for child_idx in cluster_elements[root_B]:
            child_isl = islands_list[child_idx]
            for f in child_isl.faces:
                for l in f.loops:
                    p = l[uv_layer].uv - target_centroid
                    l[uv_layer].uv = Vector((
                        p.x * cos_a - p.y * sin_a,
                        p.x * sin_a + p.y * cos_a
                    )) + anchor_centroid

        parent[root_B] = root_A
        cluster_elements[root_A].extend(cluster_elements[root_B])
        cluster_elements[root_B] = []

def _cluster_loops_by_uv(loops, uv_layer, threshold=1e-5):
    clusters = []
    for loop in loops:
        uv = loop[uv_layer].uv.copy()
        placed = False
        for cluster in clusters:
            if (cluster['uv'] - uv).length <= threshold:
                cluster['loops'].append(loop)
                cluster['uv'] = sum((item[uv_layer].uv for item in cluster['loops']), Vector((0.0, 0.0))) / len(cluster['loops'])
                placed = True
                break
        if not placed:
            clusters.append({'uv': uv, 'loops': [loop]})
    return clusters

def _collect_internal_seam_components(island):
    island_faces = set(island.faces)
    seam_edges = set()

    for face in island.faces:
        for edge in face.edges:
            if not edge.seam:
                continue
            if sum(1 for linked_face in edge.link_faces if linked_face in island_faces) == 2:
                seam_edges.add(edge)

    components = []
    visited = set()
    for edge in seam_edges:
        if edge in visited:
            continue

        stack = [edge]
        component_verts = set()
        while stack:
            current = stack.pop()
            if current in visited:
                continue

            visited.add(current)
            component_verts.update(current.verts)

            for vert in current.verts:
                for linked_edge in vert.link_edges:
                    if linked_edge in seam_edges and linked_edge not in visited:
                        stack.append(linked_edge)

        if component_verts:
            components.append(component_verts)

    return components

def _pick_twin_clusters(clusters):
    best_pair = None
    best_distance = -1.0
    for idx_a in range(len(clusters)):
        for idx_b in range(idx_a + 1, len(clusters)):
            distance = (clusters[idx_b]['uv'] - clusters[idx_a]['uv']).length_squared
            if distance > best_distance:
                best_distance = distance
                best_pair = (clusters[idx_a], clusters[idx_b])
    return best_pair

def _choose_seam_align_axis(seam_pairs):
    if not seam_pairs:
        return None

    avg_du = sum(abs(pair['b']['uv'].x - pair['a']['uv'].x) for pair in seam_pairs) / len(seam_pairs)
    avg_dv = sum(abs(pair['b']['uv'].y - pair['a']['uv'].y) for pair in seam_pairs) / len(seam_pairs)

    # Align the axis where the twin pairs are already closest.
    if abs(avg_du - avg_dv) > 1e-6:
        return 0 if avg_du < avg_dv else 1

    seam_centers = [(pair['a']['uv'] + pair['b']['uv']) * 0.5 for pair in seam_pairs]
    min_u = min(center.x for center in seam_centers)
    max_u = max(center.x for center in seam_centers)
    min_v = min(center.y for center in seam_centers)
    max_v = max(center.y for center in seam_centers)
    return 1 if (max_v - min_v) > (max_u - min_u) else 0

def _build_split_seam_pairs(island, seam_component_verts, uv_layer, threshold=1e-5):
    island_faces = set(island.faces)
    seam_pairs = []

    for vert in seam_component_verts:
        loops = [loop for loop in vert.link_loops if loop.face in island_faces]
        clusters = _cluster_loops_by_uv(loops, uv_layer, threshold)
        if len(clusters) < 2:
            continue

        twin_pair = _pick_twin_clusters(clusters)
        if twin_pair is None:
            continue

        cluster_a, cluster_b = twin_pair
        if (cluster_b['uv'] - cluster_a['uv']).length <= threshold:
            continue

        seam_pairs.append({'a': cluster_a, 'b': cluster_b})

    return seam_pairs, _choose_seam_align_axis(seam_pairs)

def _align_seam_pairs_on_axis(seam_pairs, axis_index, uv_layer):
    if axis_index is None:
        return 0

    aligned_pairs = 0
    for pair in seam_pairs:
        axis_pos = (pair['a']['uv'][axis_index] + pair['b']['uv'][axis_index]) * 0.5

        for loop in pair['a']['loops']:
            loop[uv_layer].uv[axis_index] = axis_pos

        for loop in pair['b']['loops']:
            loop[uv_layer].uv[axis_index] = axis_pos

        aligned_pairs += 1

    return aligned_pairs

def align_split_seams_in_island(uv_layer, island):
    aligned_pairs = 0
    for seam_component_verts in _collect_internal_seam_components(island):
        seam_pairs, axis_index = _build_split_seam_pairs(island, seam_component_verts, uv_layer)
        aligned_pairs += _align_seam_pairs_on_axis(seam_pairs, axis_index, uv_layer)
    return aligned_pairs

def _pick_primary_uv_cluster(loops, uv_layer):
    clusters = _cluster_loops_by_uv(loops, uv_layer)
    if not clusters:
        return None
    return max(clusters, key=lambda cluster: len(cluster['loops']))

def _build_inter_island_seam_pairs(islands_list, link, uv_layer, threshold=1e-5):
    island_a = islands_list[link['isl_a']]
    island_b = islands_list[link['isl_b']]
    seam_pairs = []

    for vert_id in sorted(set(link['shared_verts'])):
        loops_a = [loop for face in island_a.faces for loop in face.loops if loop.vert.index == vert_id]
        loops_b = [loop for face in island_b.faces for loop in face.loops if loop.vert.index == vert_id]
        if not loops_a or not loops_b:
            continue

        cluster_a = _pick_primary_uv_cluster(loops_a, uv_layer)
        cluster_b = _pick_primary_uv_cluster(loops_b, uv_layer)
        if cluster_a is None or cluster_b is None:
            continue
        if (cluster_b['uv'] - cluster_a['uv']).length <= threshold:
            continue

        seam_pairs.append({'a': cluster_a, 'b': cluster_b})

    return seam_pairs

def align_split_seams_between_islands(islands_list, links, uv_layer):
    aligned_pairs = 0
    for link in links:
        seam_pairs = _build_inter_island_seam_pairs(islands_list, link, uv_layer)
        axis_index = _choose_seam_align_axis(seam_pairs)
        aligned_pairs += _align_seam_pairs_on_axis(seam_pairs, axis_index, uv_layer)
    return aligned_pairs


def _find_island_link_components(islands_list, valid_links):
    adjacency = {isl.index: set() for isl in islands_list}
    for link in valid_links:
        adjacency[link['isl_a']].add(link['isl_b'])
        adjacency[link['isl_b']].add(link['isl_a'])

    components = []
    visited = set()
    for isl in islands_list:
        if isl.index in visited:
            continue

        stack = [isl.index]
        component = set()
        while stack:
            current = stack.pop()
            if current in visited:
                continue

            visited.add(current)
            component.add(current)
            stack.extend(adjacency[current] - visited)

        components.append(component)

    return components

def _collect_root_frontier_links(target_id, placed_ids, valid_links):
    frontier_links = []
    total_weight = 0.0

    for link in valid_links:
        idx_A = link['isl_a']
        idx_B = link['isl_b']

        if idx_A in placed_ids and idx_B == target_id:
            frontier_links.append((idx_A, idx_B, link))
            total_weight += link['shared_length']
        elif idx_B in placed_ids and idx_A == target_id:
            frontier_links.append((idx_B, idx_A, link))
            total_weight += link['shared_length']

    return frontier_links, total_weight

def _compute_frontier_transform(frontier_links, islands_list, uv_layer):
    if not frontier_links:
        return None

    delta_angle = _compute_weighted_frontier_rotation(frontier_links, islands_list, uv_layer)
    if delta_angle is None:
        strongest_link = max(frontier_links, key=lambda item: item[2]['shared_length'])
        delta_angle = _compute_link_rotation_delta(
            islands_list[strongest_link[0]],
            islands_list[strongest_link[1]],
            strongest_link[2],
            uv_layer
        )
    if delta_angle is None:
        return None

    anchor_uvs_list, target_uvs_list = _collect_cluster_frontier_correspondences(
        frontier_links, islands_list, uv_layer
    )
    if not anchor_uvs_list or len(anchor_uvs_list) != len(target_uvs_list):
        strongest_link = max(frontier_links, key=lambda item: item[2]['shared_length'])
        anchor_uvs_list, target_uvs_list = _collect_shared_uv_correspondences(
            islands_list[strongest_link[0]],
            islands_list[strongest_link[1]],
            set(strongest_link[2]['shared_verts']),
            uv_layer
        )
    if not anchor_uvs_list or len(anchor_uvs_list) != len(target_uvs_list):
        return None

    count = len(anchor_uvs_list)
    anchor_centroid = sum(anchor_uvs_list, Vector((0.0, 0.0))) / count
    target_centroid = sum(target_uvs_list, Vector((0.0, 0.0))) / count
    return delta_angle, anchor_centroid, target_centroid

def _apply_root_anchored_transform(island, uv_layer, delta_angle, anchor_centroid, target_centroid):
    cos_a = math.cos(delta_angle)
    sin_a = math.sin(delta_angle)

    for f in island.faces:
        for l in f.loops:
            p = l[uv_layer].uv - target_centroid
            l[uv_layer].uv = Vector((
                p.x * cos_a - p.y * sin_a,
                p.x * sin_a + p.y * cos_a
            )) + anchor_centroid

def align_connected_islands(islands_list, links, uv_layer):
    """
    Root-anchored final docking.
    Each connected component chooses a stable root island and then places the
    remaining islands once, against the already placed set, instead of repeatedly
    merging clusters. This reduces cumulative yaw/drift and avoids detached shells.
    """
    singleton_clusters = {isl.index: [isl.index] for isl in islands_list}
    valid_links = []
    for link in sorted(links, key=lambda item: item['shared_length'], reverse=True):
        isl_A = islands_list[link['isl_a']]
        isl_B = islands_list[link['isl_b']]
        if _is_valid_island_contact(
            link, isl_A, isl_B, singleton_clusters, islands_list, isl_A.index, isl_B.index
        ):
            valid_links.append(link)

    for component in _find_island_link_components(islands_list, valid_links):
        if len(component) < 2:
            continue

        root_id = max(component, key=lambda idx: islands_list[idx].area)
        placed_ids = {root_id}
        unplaced_ids = set(component) - placed_ids

        skipped_ids = set()
        while unplaced_ids:
            best_target_id = None
            best_frontier_links = []
            best_weight = -1.0

            for target_id in unplaced_ids:
                if target_id in skipped_ids:
                    continue
                frontier_links, total_weight = _collect_root_frontier_links(target_id, placed_ids, valid_links)
                if total_weight > best_weight and frontier_links:
                    best_target_id = target_id
                    best_frontier_links = frontier_links
                    best_weight = total_weight

            if best_target_id is None:
                break

            transform = _compute_frontier_transform(best_frontier_links, islands_list, uv_layer)
            if transform is None:
                skipped_ids.add(best_target_id)
                continue

            _apply_root_anchored_transform(
                islands_list[best_target_id], uv_layer, transform[0], transform[1], transform[2]
            )
            placed_ids.add(best_target_id)
            unplaced_ids.remove(best_target_id)
            skipped_ids.clear()
def normalize_uvs_to_origin(bm, uv_layer):
    limit = UV_RANGE_LIMIT
    min_u, max_u, min_v, max_v = 1e9, -1e9, 1e9, -1e9
    has_uvs = False
    for f in bm.faces:
        if f.select:
            for l in f.loops:
                uv = l[uv_layer].uv
                min_u, max_u = min(min_u, uv.x), max(max_u, uv.x)
                min_v, max_v = min(min_v, uv.y), max(max_v, uv.y)
                has_uvs = True
    if not has_uvs: return

    center_u, center_v = (min_u + max_u) / 2.0, (min_v + max_v) / 2.0
    shift_vec = Vector((0.0, 0.0))
    if abs(center_u) > limit: shift_vec.x = -round(center_u)
    if abs(center_v) > limit: shift_vec.y = -round(center_v)
    
    if shift_vec.length_squared > 0:
        for f in bm.faces:
            if f.select:
                for l in f.loops: l[uv_layer].uv += shift_vec

# ============================================================
# OPERATORS
# ============================================================

class HOTSPOTUV_OT_UnwrapFaces(bpy.types.Operator):
    bl_idname = "hotspotuv.unwrap_faces"
    bl_label = "UV Unwrap Faces"
    bl_description = "Two-Pass Unwrap: Pins selected core faces and seamlessly relaxes unselected chamfers."
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _apply_settings_to_globals(context.scene.hotspotuv_settings)
        
        # Валидация
        valid, error, bm = validate_edit_mesh(context, require_selection=True, selection_type='FACE')
        if not valid:
            self.report({"WARNING"}, error)
            return {"CANCELLED"}
        
        mesh = context.edit_object.data
        sel_faces = [f for f in bm.faces if f.select]
        
        # Запоминаем оригинальные швы
        original_seams = [e.seam for e in bm.edges]
        
        try:
            # Создаем временные швы на sharp edges
            for e in bm.edges:
                if not e.smooth: e.seam = True
                
            # 1. АНАЛИЗ ВЫДЕЛЕНИЯ (Ядро vs Полный лоскут)
            islands_data = get_expanded_islands(bm, sel_faces)
            
            islands_indices = []
            for data in islands_data:
                islands_indices.append({
                    'full': [f.index for f in data['full']],
                    'core': [f.index for f in data['core']]
                })

            # 2. ПЕРВЫЙ ПРОХОД (UNWRAP ТОЛЬКО ЯДЕР)
            for f in bm.faces: f.select = False
            for data_idx in islands_indices:
                for i in data_idx['core']: bm.faces[i].select = True
            bmesh.update_edit_mesh(mesh)
            
            bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)
            
            # Обновляем BMesh и прибиваем Ядра гвоздями (Pin)
            bm.free()
            bm = bmesh.from_edit_mesh(mesh)
            bm.faces.ensure_lookup_table(); bm.verts.ensure_lookup_table(); bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()
            
            for data_idx in islands_indices:
                core_faces = [bm.faces[i] for i in data_idx['core']]
                if not core_faces: continue
                
                core_island = IslandInfo(core_faces, 0)
                analyze_island_properties(core_island)
                
                # Ставим идеальный скейл, оффсет и поворот для Ядра
                orient_scale_and_position_island(uv_layer, core_island)
                
                # Замораживаем
                for f in core_faces:
                    for l in f.loops: l[uv_layer].pin_uv = True

            # 3. ВТОРОЙ ПРОХОД (ДОРАЗВЕРТКА ФАСОК)
            for f in bm.faces: f.select = False
            for data_idx in islands_indices:
                for i in data_idx['full']: bm.faces[i].select = True
            bmesh.update_edit_mesh(mesh)
            
            # Blender сам подтянет фаски к запиненным Ядрам
            bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)
            
            bm.free()
            bm = bmesh.from_edit_mesh(mesh)
            bm.faces.ensure_lookup_table(); bm.verts.ensure_lookup_table(); bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()

            # 4. ОЧИСТКА И ДОКИНГ
            for i, e in enumerate(bm.edges): e.seam = original_seams[i]
            for f in bm.faces:
                for l in f.loops: l[uv_layer].pin_uv = False
                
            final_islands = []
            for idx, data_idx in enumerate(islands_indices):
                full_faces = [bm.faces[i] for i in data_idx['full']]
                isl = IslandInfo(full_faces, idx)
                analyze_island_properties(isl)
                final_islands.append(isl)
                
            links = build_edge_based_links(final_islands, bm)
            
            align_connected_islands(final_islands, links, uv_layer)
            align_split_seams_between_islands(final_islands, links, uv_layer)
            for isl in final_islands:
                align_split_seams_in_island(uv_layer, isl)
            normalize_uvs_to_origin(bm, uv_layer)
            
            # Возвращаем выделение как было у пользователя
            for f in bm.faces: f.select = False
            for f_idx in [i for d in islands_indices for i in d['core']]:
                bm.faces[f_idx].select = True
                
            bmesh.update_edit_mesh(mesh)
            self.report({"INFO"}, "Two-Pass Unwrap: Cores positioned absolutely, chamfers seamlessly expanded.")
            return {"FINISHED"}
            
        except Exception as e:
            # Восстанавливаем seams при ошибке
            try:
                bm = bmesh.from_edit_mesh(mesh)
                bm.edges.ensure_lookup_table()
                for i, edge in enumerate(bm.edges):
                    if i < len(original_seams):
                        edge.seam = original_seams[i]
                bmesh.update_edit_mesh(mesh)
            except:
                pass
            self.report({"ERROR"}, f"Unwrap failed: {str(e)}")
            return {"CANCELLED"}

# ============================================================
# UTILITY TOOLS
# ============================================================

class HOTSPOTUV_OT_ManualDock(bpy.types.Operator):
    bl_idname = "hotspotuv.manual_dock"
    bl_label = "Manual Dock Islands"
    bl_description = "Dock UV islands based on selected boundary edges (sharp/seam)."
    bl_options = {"REGISTER", "UNDO"}

    direction: EnumProperty(
        name="Direction",
        items=[
            ('AUTO', 'Auto', 'Larger 3D area island becomes root anchor'),
            ('REVERSE', 'Reverse', 'Smaller 3D area island becomes root anchor')
        ],
        default='AUTO'
    )
    
    fit_vertices: BoolProperty(
        name="Fit Vertices",
        description="Move target edge vertices to match anchor edge positions",
        default=True
    )
    
    unwrap_interior: BoolProperty(
        name="Unwrap Interior (Conformal)",
        description="Relax the rest of the island while keeping fitted vertices pinned",
        default=False
    )

    # Отрисовка UI, где Unwrap активен только при включенном Fit Vertices
    def draw(self, context):
        layout = self.layout
        layout.prop(self, "direction")
        layout.prop(self, "fit_vertices")
        
        col = layout.column()
        col.enabled = self.fit_vertices
        col.prop(self, "unwrap_interior")

    def execute(self, context):
        valid, error, bm = validate_edit_mesh(context, require_selection=True, selection_type='EDGE')
        if not valid:
            self.report({"WARNING"}, error)
            return {"CANCELLED"}
        
        try:
            # Сохраняем выделение рёбер для восстановления
            orig_edge_sel = [e.index for e in bm.edges if e.select]
            
            sel_edges = [e for e in bm.edges if e.select and (not e.smooth or e.seam)]
            
            if not sel_edges:
                self.report({"WARNING"}, "No boundary edges selected (must be sharp or seam)")
                return {"CANCELLED"}
            
            islands, graph, face_to_island = build_island_graph(sel_edges, bm)
            if not islands:
                self.report({"WARNING"}, "No valid islands found")
                return {"CANCELLED"}
            
            # Запускаем стыковку ВСЕХ независимых цепочек
            # Unwrap происходит ПОСЛОЙНО внутри dock_all_chains
            docked_count, bm = dock_all_chains(
                islands, graph, bm, context, self.direction, self.fit_vertices, self.unwrap_interior
            )
            
            if docked_count == 0:
                self.report({"WARNING"}, "No island pairs found for docking")
                return {"CANCELLED"}
            
            # Восстанавливаем выделение рёбер
            for f in bm.faces:
                f.select = False
            for e_idx in orig_edge_sel:
                if e_idx < len(bm.edges):
                    bm.edges[e_idx].select = True

            bmesh.update_edit_mesh(context.edit_object.data)
            self.report({"INFO"}, f"Docked {docked_count} island(s) across all chains")
            return {"FINISHED"}
            
        except Exception as e:
            self.report({"ERROR"}, f"Docking failed: {str(e)}")
            return {"CANCELLED"}

class HOTSPOTUV_OT_SelectSimilar(bpy.types.Operator):
    bl_idname = "hotspotuv.select_similar"
    bl_label = "Select Similar Islands"
    bl_description = "Selects all islands in the mesh with the same 3D area as the current selection."
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        valid, error, bm = validate_edit_mesh(context, require_selection=True, selection_type='FACE')
        if not valid:
            self.report({"WARNING"}, error)
            return {"CANCELLED"}
        
        try:
            sel_faces = set(f for f in bm.faces if f.select)
            
            visible_faces = [f for f in bm.faces if not f.hide]
            islands = []
            for idx, group in enumerate(get_expanded_islands(bm, visible_faces)):
                isl = IslandInfo(group['full'], idx)
                analyze_island_properties(isl)
                islands.append(isl)
                
            target_areas = [isl.area for isl in islands if any(f in sel_faces for f in isl.faces)]
            if not target_areas:
                self.report({"WARNING"}, "Could not determine target area from selection")
                return {"CANCELLED"}
                
            matched_count = 0
            for isl in islands:
                if any(t_area > 0 and abs(isl.area - t_area) / t_area <= 0.02 for t_area in target_areas):
                    matched_count += 1
                    for f in isl.faces: f.select = True
            
            bmesh.update_edit_mesh(context.edit_object.data)
            self.report({"INFO"}, f"Selected {matched_count} similar islands.")
            return {"FINISHED"}
            
        except Exception as e:
            self.report({"ERROR"}, f"Select similar failed: {str(e)}")
            return {"CANCELLED"}

class HOTSPOTUV_OT_StackSimilar(bpy.types.Operator):
    bl_idname = "hotspotuv.stack_similar"
    bl_label = "Stack Similar Islands"
    bl_description = "Groups selected islands by area and perfectly aligns them with 4-way rotation lock."
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _apply_settings_to_globals(context.scene.hotspotuv_settings)
        
        valid, error, bm = validate_edit_mesh(context, require_selection=True, selection_type='FACE')
        if not valid:
            self.report({"WARNING"}, error)
            return {"CANCELLED"}
        
        try:
            sel_faces = [f for f in bm.faces if f.select]
            
            islands = []
            for idx, group in enumerate(get_expanded_islands(bm, sel_faces)):
                isl = IslandInfo(group['full'], idx)
                analyze_island_properties(isl)
                islands.append(isl)
                
            uv_layer = bm.loops.layers.uv.verify()
            islands.sort(key=lambda x: x.area, reverse=True)
            
            groups, current_group = [], []
            for isl in islands:
                if not current_group or (current_group[0].area > 0 and abs(isl.area - current_group[0].area) / current_group[0].area <= 0.02):
                    current_group.append(isl)
                else:
                    groups.append(current_group)
                    current_group = [isl]
            if current_group: groups.append(current_group)

            def get_centered_unique_uvs(island):
                uvs, center, count = [], Vector((0.0, 0.0)), 0
                for f in island.faces:
                    for l in f.loops:
                        uvs.append(l[uv_layer].uv.copy())
                        center += l[uv_layer].uv
                        count += 1
                if count == 0: return [], Vector((0.0, 0.0))
                center /= count
                unique_uvs = []
                for uv in uvs:
                    uv_centered = uv - center
                    if not any((uv_centered - u).length_squared < 1e-5 for u in unique_uvs): unique_uvs.append(uv_centered)
                return unique_uvs, center

            stacked_count = 0
            for group in groups:
                if len(group) < 2: continue
                anchor_uvs, anchor_center = get_centered_unique_uvs(group[0])
                if not anchor_uvs: continue
                
                for i in range(1, len(group)):
                    source_isl = group[i]
                    source_uvs, source_center = get_centered_unique_uvs(source_isl)
                    if not source_uvs: continue
                    
                    best_angle, min_err = 0.0, float('inf')
                    for angle in [0.0, math.pi/2, math.pi, 3*math.pi/2]:
                        err, cos_a, sin_a = 0.0, math.cos(angle), math.sin(angle)
                        for suv in source_uvs:
                            rx, ry = suv.x * cos_a - suv.y * sin_a, suv.x * sin_a + suv.y * cos_a
                            err += min((rx - auv.x)**2 + (ry - auv.y)**2 for auv in anchor_uvs)
                        if err < min_err:
                            min_err, best_angle = err, angle
                            
                    cos_a, sin_a = math.cos(best_angle), math.sin(best_angle)
                    for f in source_isl.faces:
                        for l in f.loops:
                            uv = l[uv_layer].uv - source_center
                            rx, ry = uv.x * cos_a - uv.y * sin_a, uv.x * sin_a + uv.y * cos_a
                            l[uv_layer].uv = Vector((rx, ry)) + anchor_center
                    stacked_count += 1
                    
            bmesh.update_edit_mesh(context.edit_object.data)
            self.report({"INFO"}, f"Stacked {stacked_count} identical islands.")
            return {"FINISHED"}
            
        except Exception as e:
            self.report({"ERROR"}, f"Stack similar failed: {str(e)}")
            return {"CANCELLED"}

# ============================================================
# PANEL
# ============================================================

class HOTSPOTUV_PT_Panel(bpy.types.Panel):
    bl_label = "Hotspot UV"
    bl_idname = "HOTSPOTUV_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Hotspot UV"

    def draw(self, context):
        layout = self.layout
        s = context.scene.hotspotuv_settings
        col = layout.column(align=True)
        col.prop(s, "target_texel_density")
        col.prop(s, "texture_size")
        col.prop(s, "uv_scale")
        col.prop(s, "uv_range_limit")
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Face Tools:")
        col.operator("hotspotuv.unwrap_faces", text="UV Unwrap Faces", icon="UV")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Edge Tools:")
        col.operator("hotspotuv.manual_dock", text="Manual Dock Islands", icon="SNAP_ON")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Utility Tools:")
        col.operator("hotspotuv.select_similar", text="Select Similar Islands", icon="RESTRICT_SELECT_OFF")
        col.operator("hotspotuv.stack_similar", text="Stack Similar Islands", icon="ALIGN_CENTER")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Debug:")
        
        # Analyze toggle button
        if s.dbg_active:
            col.operator("hotspotuv.debug_analysis", text="Analyze: ON", icon="PAUSE", depress=True)
        else:
            col.operator("hotspotuv.debug_analysis", text="Analyze: OFF", icon="VIEWZOOM")
        
        # Layer controls — only visible when debug is active
        if s.dbg_active and s.dbg_source_object:
            gp_name = GP_DEBUG_PREFIX + s.dbg_source_object
            gp_obj = bpy.data.objects.get(gp_name)
            has_gp = gp_obj is not None and gp_obj.type == 'GPENCIL'
            
            if has_gp:
                gp_data = gp_obj.data
                
                # --- Patches group (collapsible) ---
                box = col.box()
                row = box.row(align=True)
                icon = 'TRIA_DOWN' if s.dbg_grp_patches else 'TRIA_RIGHT'
                op = row.operator("hotspotuv.debug_toggle_group", text="", icon=icon, emboss=False)
                op.group_name = "patches"
                row.label(text="Patches", icon="MESH_GRID")
                if s.dbg_grp_patches:
                    for ptype in ('WALL', 'FLOOR', 'SLOPE'):
                        layer_name = f"Patches_{ptype}"
                        if layer_name in gp_data.layers:
                            layer = gp_data.layers[layer_name]
                            row = box.row(align=True)
                            row.separator(factor=2.0)
                            icon = 'HIDE_OFF' if not layer.hide else 'HIDE_ON'
                            op = row.operator("hotspotuv.debug_toggle_layer", text="", icon=icon)
                            op.layer_name = layer_name
                            row.label(text=ptype.capitalize())
                
                # --- Frame group (collapsible) ---
                box = col.box()
                row = box.row(align=True)
                icon = 'TRIA_DOWN' if s.dbg_grp_frame else 'TRIA_RIGHT'
                op = row.operator("hotspotuv.debug_toggle_group", text="", icon=icon, emboss=False)
                op.group_name = "frame"
                row.label(text="Frame", icon="MOD_WIREFRAME")
                if s.dbg_grp_frame:
                    for fname, label in [('Frame_H', 'Horizontal'), ('Frame_V', 'Vertical'),
                                         ('Frame_FREE', 'Free'), ('Frame_HOLE', 'Holes')]:
                        if fname in gp_data.layers:
                            layer = gp_data.layers[fname]
                            row = box.row(align=True)
                            row.separator(factor=2.0)
                            icon = 'HIDE_OFF' if not layer.hide else 'HIDE_ON'
                            op = row.operator("hotspotuv.debug_toggle_layer", text="", icon=icon)
                            op.layer_name = fname
                            row.label(text=label)
                
                # --- Loop Types group (collapsible) ---
                box = col.box()
                row = box.row(align=True)
                icon = 'TRIA_DOWN' if s.dbg_grp_loops else 'TRIA_RIGHT'
                op = row.operator("hotspotuv.debug_toggle_group", text="", icon=icon, emboss=False)
                op.group_name = "loops"
                row.label(text="Loop Types", icon="CURVE_BEZCIRCLE")
                if s.dbg_grp_loops:
                    for lname, label in [('Loops_Chains', 'Chains'), ('Loops_Boundary', 'Boundary'),
                                         ('Loops_Holes', 'Holes')]:
                        if lname in gp_data.layers:
                            layer = gp_data.layers[lname]
                            row = box.row(align=True)
                            row.separator(factor=2.0)
                            icon = 'HIDE_OFF' if not layer.hide else 'HIDE_ON'
                            op = row.operator("hotspotuv.debug_toggle_layer", text="", icon=icon)
                            op.layer_name = lname
                            row.label(text=label)
                
                # --- Overlay group (collapsible) ---
                box = col.box()
                row = box.row(align=True)
                icon = 'TRIA_DOWN' if s.dbg_grp_overlay else 'TRIA_RIGHT'
                op = row.operator("hotspotuv.debug_toggle_group", text="", icon=icon, emboss=False)
                op.group_name = "overlay"
                row.label(text="Overlay", icon="ORIENTATION_LOCAL")
                if s.dbg_grp_overlay:
                    layer_name = "Overlay_Basis"
                    if layer_name in gp_data.layers:
                        layer = gp_data.layers[layer_name]
                        row = box.row(align=True)
                        row.separator(factor=2.0)
                        icon = 'HIDE_OFF' if not layer.hide else 'HIDE_ON'
                        op = row.operator("hotspotuv.debug_toggle_layer", text="", icon=icon)
                        op.layer_name = layer_name
                        row.label(text="Basis (U/V/N)")
            
            col.separator()
            col.operator("hotspotuv.debug_clear", text="Force Clear", icon="X")

classes = (HOTSPOTUV_Settings, HOTSPOTUV_OT_UnwrapFaces, HOTSPOTUV_OT_ManualDock, HOTSPOTUV_OT_SelectSimilar, HOTSPOTUV_OT_StackSimilar, HOTSPOTUV_OT_DebugAnalysis, HOTSPOTUV_OT_DebugClear, HOTSPOTUV_OT_DebugToggleLayer, HOTSPOTUV_OT_DebugToggleGroup, HOTSPOTUV_PT_Panel)

def register():
    for cls in classes: bpy.utils.register_class(cls)
    bpy.types.Scene.hotspotuv_settings = PointerProperty(type=HOTSPOTUV_Settings)

def unregister():
    # Clean up any GP debug objects
    for obj in list(bpy.data.objects):
        if obj.name.startswith(GP_DEBUG_PREFIX):
            bpy.data.objects.remove(obj, do_unlink=True)
    if hasattr(bpy.types.Scene, "hotspotuv_settings"): del bpy.types.Scene.hotspotuv_settings
    for cls in reversed(classes): bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()