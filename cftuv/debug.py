import bpy
import colorsys

try:
    from .constants import GP_DEBUG_PREFIX
    from .model import FrameRole, LoopKind, PatchGraph, PatchType
except ImportError:
    from constants import GP_DEBUG_PREFIX
    from model import FrameRole, LoopKind, PatchGraph, PatchType


_GP_STYLES = {
    'Frame_H': (1.0, 0.85, 0.0, 1.0),
    'Frame_V': (0.0, 0.85, 0.85, 1.0),
    'Frame_FREE': (0.5, 0.5, 0.5, 0.6),
    'Frame_HOLE': (0.2, 0.2, 0.6, 0.8),
}

_BASIS_COLORS = {
    'U': (1.0, 0.15, 0.15, 1.0),
    'V': (0.15, 1.0, 0.15, 1.0),
    'N': (0.2, 0.2, 1.0, 1.0),
}


def _enum_value(value):
    return value.value if hasattr(value, 'value') else value


def _get_gp_debug_name(source_obj):
    return GP_DEBUG_PREFIX + source_obj.name


def _get_or_create_gp_object(source_obj):
    gp_name = _get_gp_debug_name(source_obj)

    if gp_name in bpy.data.objects:
        gp_obj = bpy.data.objects[gp_name]
        if gp_obj.type == 'GPENCIL':
            return gp_obj
        bpy.data.objects.remove(gp_obj, do_unlink=True)

    gp_data = bpy.data.grease_pencils.new(gp_name)
    gp_obj = bpy.data.objects.new(gp_name, gp_data)
    bpy.context.scene.collection.objects.link(gp_obj)
    gp_obj.matrix_world = source_obj.matrix_world.copy()
    gp_data.stroke_depth_order = '3D'
    gp_data.stroke_thickness_space = 'SCREENSPACE'
    return gp_obj


def _ensure_gp_layer(gp_data, layer_name, color_rgba):
    mat_name = f"CFTUV_{layer_name}"
    if mat_name in bpy.data.materials:
        mat = bpy.data.materials[mat_name]
    else:
        mat = bpy.data.materials.new(mat_name)
        bpy.data.materials.create_gpencil_data(mat)

    mat.grease_pencil.color = color_rgba[:4]
    mat.grease_pencil.show_fill = False

    mat_idx = None
    for i, slot in enumerate(gp_data.materials):
        if slot and slot.name == mat_name:
            mat_idx = i
            break
    if mat_idx is None:
        gp_data.materials.append(mat)
        mat_idx = len(gp_data.materials) - 1

    if layer_name in gp_data.layers:
        layer = gp_data.layers[layer_name]
        layer.clear()
    else:
        layer = gp_data.layers.new(layer_name, set_active=False)

    if not layer.frames:
        frame = layer.frames.new(0)
    else:
        frame = layer.frames[0]

    return frame, mat_idx


def _add_gp_stroke(frame, points, mat_idx, line_width=4):
    if len(points) < 2:
        return
    stroke = frame.strokes.new()
    stroke.material_index = mat_idx
    stroke.line_width = line_width
    stroke.points.add(len(points))
    for i, point in enumerate(points):
        stroke.points[i].co = (point.x, point.y, point.z)
        stroke.points[i].strength = 1.0
        stroke.points[i].pressure = 1.0


def clear_visualization(source_obj):
    gp_name = _get_gp_debug_name(source_obj)
    if gp_name in bpy.data.objects:
        obj = bpy.data.objects[gp_name]
        bpy.data.objects.remove(obj, do_unlink=True)
    if gp_name in bpy.data.grease_pencils:
        bpy.data.grease_pencils.remove(bpy.data.grease_pencils[gp_name])

    mesh_name = GP_DEBUG_PREFIX + "Mesh_" + source_obj.name
    if mesh_name in bpy.data.objects:
        obj = bpy.data.objects[mesh_name]
        mesh_data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh_data and mesh_data.users == 0:
            bpy.data.meshes.remove(mesh_data)

    for mat in list(bpy.data.materials):
        if (
            mat.name.startswith("CFTUV_P")
            or mat.name.startswith("CFTUV_Axis_")
            or mat.name.startswith("CFTUV_Ch")
            or mat.name.startswith("CFTUV_Lp")
            or mat.name.startswith("CFTUV_Hl")
        ) and mat.users == 0:
            bpy.data.materials.remove(mat)


def _create_patch_mesh(graph: PatchGraph, source_obj):
    mesh_name = GP_DEBUG_PREFIX + "Mesh_" + source_obj.name

    if mesh_name in bpy.data.objects:
        old_obj = bpy.data.objects[mesh_name]
        old_data = old_obj.data
        bpy.data.objects.remove(old_obj, do_unlink=True)
        if old_data and old_data.users == 0:
            bpy.data.meshes.remove(old_data)

    all_verts = []
    all_faces = []
    face_mat_indices = []
    golden_ratio = 0.618033988749895
    materials = []

    patch_ids = sorted(graph.nodes.keys())
    for draw_idx, patch_id in enumerate(patch_ids):
        node = graph.nodes[patch_id]
        offset = len(all_verts)

        for vert in node.mesh_verts:
            all_verts.append(vert)

        for tri in node.mesh_tris:
            all_faces.append(tuple(index + offset for index in tri))
            face_mat_indices.append(draw_idx)

        hue = (draw_idx * golden_ratio) % 1.0
        patch_type = _enum_value(node.patch_type)
        if patch_type == PatchType.WALL.value:
            sat, val, alpha = 0.8, 0.9, 0.4
        else:
            sat, val, alpha = 0.5, 0.6, 0.3
        r, g, b = colorsys.hsv_to_rgb(hue, sat, val)

        mat_name = f"CFTUV_P{draw_idx:03d}"
        if mat_name in bpy.data.materials:
            mat = bpy.data.materials[mat_name]
        else:
            mat = bpy.data.materials.new(mat_name)
        mat.diffuse_color = (r, g, b, alpha)
        mat.use_nodes = False
        materials.append(mat)

    mesh_data = bpy.data.meshes.new(mesh_name)
    mesh_data.from_pydata([vert[:] for vert in all_verts], [], all_faces)
    mesh_data.update()

    for mat in materials:
        mesh_data.materials.append(mat)

    for face_index, mat_index in enumerate(face_mat_indices):
        if face_index < len(mesh_data.polygons):
            mesh_data.polygons[face_index].material_index = mat_index

    mesh_obj = bpy.data.objects.new(mesh_name, mesh_data)
    mesh_obj.matrix_world = source_obj.matrix_world.copy()
    bpy.context.scene.collection.objects.link(mesh_obj)
    mesh_obj.show_transparent = True
    mesh_obj.display_type = 'SOLID'
    mesh_obj.color = (1, 1, 1, 0.5)
    mesh_obj.hide_viewport = True
    mesh_obj.hide_render = True


def _apply_layer_visibility(gp_data, dbg_settings):
    grp_patches = dbg_settings.get('grp_patches', True)
    grp_frame = dbg_settings.get('grp_frame', True)
    grp_loops = dbg_settings.get('grp_loops', True)
    grp_overlay = dbg_settings.get('grp_overlay', True)

    mapping = {
        'Patches_WALL': grp_patches and dbg_settings.get('patches_wall', True),
        'Patches_FLOOR': grp_patches and dbg_settings.get('patches_floor', True),
        'Patches_SLOPE': grp_patches and dbg_settings.get('patches_slope', True),
        'Frame_H': grp_frame and dbg_settings.get('frame_h', True),
        'Frame_V': grp_frame and dbg_settings.get('frame_v', True),
        'Frame_FREE': grp_frame and dbg_settings.get('frame_free', True),
        'Frame_HOLE': grp_frame and dbg_settings.get('frame_hole', True),
        'Loops_Chains': grp_loops,
        'Loops_Boundary': grp_loops,
        'Loops_Holes': grp_loops,
        'Overlay_Basis': grp_overlay,
    }
    for layer_name, visible in mapping.items():
        if layer_name in gp_data.layers:
            gp_data.layers[layer_name].hide = not visible


def create_visualization(graph: PatchGraph, source_obj, settings_dict=None):
    gp_obj = _get_or_create_gp_object(source_obj)
    gp_data = gp_obj.data

    _create_patch_mesh(graph, source_obj)

    frames_and_mats = {}
    for style_name, color in _GP_STYLES.items():
        frame, mat_idx = _ensure_gp_layer(gp_data, style_name, color)
        frames_and_mats[style_name] = (frame, mat_idx)

    basis_layer_name = 'Overlay_Basis'
    if basis_layer_name in gp_data.layers:
        basis_layer = gp_data.layers[basis_layer_name]
        basis_layer.clear()
    else:
        basis_layer = gp_data.layers.new(basis_layer_name, set_active=False)
    basis_frame = basis_layer.frames[0] if basis_layer.frames else basis_layer.frames.new(0)

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

        mat_idx = None
        for i, slot in enumerate(gp_data.materials):
            if slot and slot.name == mat_name:
                mat_idx = i
                break
        if mat_idx is None:
            gp_data.materials.append(mat)
            mat_idx = len(gp_data.materials) - 1
        basis_mats[axis_key] = mat_idx

    loop_layer_names = ['Loops_Chains', 'Loops_Boundary', 'Loops_Holes']
    loop_layers = {}
    for layer_name in loop_layer_names:
        if layer_name in gp_data.layers:
            layer = gp_data.layers[layer_name]
            layer.clear()
        else:
            layer = gp_data.layers.new(layer_name, set_active=False)
        loop_layers[layer_name] = layer.frames[0] if layer.frames else layer.frames.new(0)

    chain_counter = 0
    loop_counter = 0
    hole_counter = 0
    golden_ratio = 0.618033988749895

    def _get_element_mat(prefix, index, hue_offset, sat, val):
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

    patch_types = {PatchType.WALL.value, PatchType.FLOOR.value, PatchType.SLOPE.value}
    patch_layers = {}
    for patch_type in patch_types:
        layer_name = f"Patches_{patch_type}"
        if layer_name in gp_data.layers:
            layer = gp_data.layers[layer_name]
            layer.clear()
        else:
            layer = gp_data.layers.new(layer_name, set_active=False)
        patch_layers[patch_type] = layer.frames[0] if layer.frames else layer.frames.new(0)

    patch_mat_indices = {}
    for draw_idx, patch_id in enumerate(sorted(graph.nodes.keys())):
        node = graph.nodes[patch_id]
        patch_type = _enum_value(node.patch_type)
        hue = (draw_idx * golden_ratio) % 1.0
        if patch_type == PatchType.WALL.value:
            sat, val = 0.8, 0.9
        elif patch_type == PatchType.SLOPE.value:
            sat, val = 0.7, 0.75
        else:
            sat, val = 0.5, 0.6
        r, g, b = colorsys.hsv_to_rgb(hue, sat, val)

        mat_name = f"CFTUV_P{draw_idx:03d}"
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
        patch_mat_indices[patch_id] = mat_idx

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        centroid = node.centroid
        axis_len = 0.15

        _add_gp_stroke(basis_frame, [centroid, centroid + node.basis_u * axis_len], basis_mats['U'], line_width=8)
        _add_gp_stroke(basis_frame, [centroid, centroid + node.basis_v * axis_len], basis_mats['V'], line_width=8)
        _add_gp_stroke(basis_frame, [centroid, centroid + node.normal * axis_len * 0.6], basis_mats['N'], line_width=6)

        patch_type = _enum_value(node.patch_type)
        if patch_type not in patch_layers:
            patch_type = PatchType.WALL.value
        patch_frame = patch_layers[patch_type]
        patch_mat = patch_mat_indices[patch_id]
        for tri in node.mesh_tris:
            if len(tri) < 3:
                continue
            stroke = patch_frame.strokes.new()
            stroke.material_index = patch_mat
            stroke.line_width = 1
            stroke.use_cyclic = True
            stroke.points.add(len(tri))
            for i, vert_index in enumerate(tri):
                point = node.mesh_verts[vert_index]
                stroke.points[i].co = (point.x, point.y, point.z)
                stroke.points[i].strength = 1.0
                stroke.points[i].pressure = 1.0

        for loop in node.boundary_loops:
            for chain in loop.chains:
                points = chain.vert_cos + [chain.vert_cos[0]] if chain.is_closed else chain.vert_cos
                if len(points) < 2:
                    continue
                kind = _enum_value(loop.kind)
                role = _enum_value(chain.frame_role)

                if kind == LoopKind.HOLE.value:
                    style, width = 'Frame_HOLE', 3
                elif role == FrameRole.H_FRAME.value:
                    style, width = 'Frame_H', 6
                elif role == FrameRole.V_FRAME.value:
                    style, width = 'Frame_V', 6
                else:
                    style, width = 'Frame_FREE', 3

                frame, mat_idx = frames_and_mats[style]
                _add_gp_stroke(frame, points, mat_idx, line_width=width)

            for chain in loop.chains:
                if len(chain.vert_cos) < 2:
                    continue
                mat_idx = _get_element_mat('Ch', chain_counter, 0.0, 0.9, 0.85)
                chain_counter += 1
                points = chain.vert_cos + [chain.vert_cos[0]] if chain.is_closed else chain.vert_cos
                _add_gp_stroke(loop_layers['Loops_Chains'], points, mat_idx, line_width=4)

            if len(loop.vert_cos) < 3:
                continue
            if _enum_value(loop.kind) == LoopKind.HOLE.value:
                mat_idx = _get_element_mat('Hl', hole_counter, 0.5, 0.6, 0.7)
                hole_counter += 1
                _add_gp_stroke(loop_layers['Loops_Holes'], loop.vert_cos + [loop.vert_cos[0]], mat_idx, line_width=5)
            else:
                mat_idx = _get_element_mat('Lp', loop_counter, 0.25, 0.85, 0.9)
                loop_counter += 1
                _add_gp_stroke(loop_layers['Loops_Boundary'], loop.vert_cos + [loop.vert_cos[0]], mat_idx, line_width=5)

    if settings_dict:
        _apply_layer_visibility(gp_data, settings_dict)

    return gp_obj


__all__ = ['create_visualization', 'clear_visualization']
