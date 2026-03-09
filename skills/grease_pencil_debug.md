# SKILL: Grease Pencil Debug Visualization

## When to use
Any work in debug.py or when verifying that refactoring didn't break visualization.

## Architecture rule

debug.py reads PatchGraph, never BMesh.
All geometry data comes from PatchNode.mesh_verts, mesh_tris, boundary_loops[].vert_cos.

## GP object structure

```
CFTUV_Debug_{object_name}     (GP object, same transform as source mesh)
├── Patches_WALL               (layer — filled triangles, per-patch material)
├── Patches_FLOOR              (layer)
├── Patches_SLOPE              (layer)
├── Frame_H                    (layer — yellow strokes, horizontal segments)
├── Frame_V                    (layer — cyan strokes, vertical segments)
├── Frame_FREE                 (layer — gray strokes)
├── Frame_HOLE                 (layer — dark blue strokes)
├── Loops_Chains               (layer — per-chain unique color)
├── Loops_Boundary             (layer — per-loop unique color, OUTER only)
├── Loops_Holes                (layer — per-loop unique color, HOLE only)
└── Overlay_Basis              (layer — RGB axes: U=red, V=green, N=blue)
```

## Creating GP strokes from PatchGraph

```python
def _draw_patch_fill(gp_data, node: PatchNode):
    """Заливка патча через триангулированные strokes."""
    layer_name = f"Patches_{node.patch_type.value}"
    # ... ensure layer, get frame, get material ...

    for tri in node.mesh_tris:
        stroke = frame.strokes.new()
        stroke.material_index = mat_idx
        stroke.line_width = 1
        stroke.use_cyclic = True
        stroke.points.add(3)
        for i, vi in enumerate(tri):
            pt = node.mesh_verts[vi]
            stroke.points[i].co = (pt.x, pt.y, pt.z)
            stroke.points[i].strength = 1.0
            stroke.points[i].pressure = 1.0

def _draw_basis_axes(gp_data, node: PatchNode):
    """RGB оси базиса в centroid патча."""
    c = node.centroid
    axis_len = 0.15
    # U axis (red)
    _add_gp_stroke(frame, [c, c + node.basis_u * axis_len], u_mat, line_width=8)
    # V axis (green)
    _add_gp_stroke(frame, [c, c + node.basis_v * axis_len], v_mat, line_width=8)
    # N axis (blue)
    _add_gp_stroke(frame, [c, c + node.normal * axis_len * 0.6], n_mat, line_width=6)
```

## Color scheme

```python
# Frame segments
'Frame_H':     (1.0, 0.85, 0.0, 1.0)    # yellow
'Frame_V':     (0.0, 0.85, 0.85, 1.0)    # cyan
'Frame_FREE':  (0.5, 0.5, 0.5, 0.6)      # gray
'Frame_HOLE':  (0.2, 0.2, 0.6, 0.8)      # dark blue

# Basis axes
'U': (1.0, 0.15, 0.15, 1.0)   # red
'V': (0.15, 1.0, 0.15, 1.0)   # green
'N': (0.2, 0.2, 1.0, 1.0)     # blue

# Patches — golden ratio hue distribution
hue = (patch_index * 0.618033988749895) % 1.0
# WALL: sat=0.8, val=0.9
# FLOOR: sat=0.5, val=0.6
# SLOPE: sat=0.7, val=0.75
```

## GP material setup

```python
mat = bpy.data.materials.new(mat_name)
bpy.data.materials.create_gpencil_data(mat)
mat.grease_pencil.color = (r, g, b, a)        # stroke color

# Для patch fill:
mat.grease_pencil.show_fill = True
mat.grease_pencil.fill_color = (r, g, b, a)
mat.grease_pencil.show_stroke = False

# Для strokes:
mat.grease_pencil.show_fill = False
```

## GP object settings

```python
gp_data.stroke_depth_order = '3D'               # 3D depth ordering
gp_data.stroke_thickness_space = 'SCREENSPACE'   # constant screen-space width
gp_obj.matrix_world = source_obj.matrix_world.copy()  # same transform
```

## Cleanup

```python
def clear_visualization(source_obj):
    """Удалить ВСЁ: GP object, debug mesh, orphan materials."""
    # 1. Remove GP object
    # 2. Remove GP data block
    # 3. Remove debug mesh object + mesh data
    # 4. Remove orphan materials (CFTUV_P*, CFTUV_Axis_*, CFTUV_Ch*, CFTUV_Lp*, CFTUV_Hl*)
```

## Panel layer toggles

Панель управляет visibility через `gp_data.layers[name].hide = True/False`.
Группы (Patches/Frame/Loops/Overlay) — collapsible boxes в UI.
Каждый layer можно показать/скрыть независимо.
