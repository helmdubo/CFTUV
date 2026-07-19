"""Headless acceptance render для GPU preview ``matrix_world``.

Запускать Blender с ``--factory-startup --background --python``. Скрипт не
сохраняет blend: строит два временных объекта с несброшенными трансформами,
показывает local-space ошибку красным и исправленный world-space overlay синим.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Euler, Matrix, Vector


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cftuv.decal_geometry import DecalGeometryFace  # noqa: E402
from cftuv.decal_gpu_preview import build_preview_buffers  # noqa: E402


OUTPUT_PNG = REPO_ROOT / "artifacts" / "decal_gpu_matrix_world_before_after.png"
OUTPUT_JSON = REPO_ROOT / "artifacts" / "decal_gpu_matrix_world_acceptance.json"


def _material(name, color, metallic=0.0, roughness=0.45):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Roughness"].default_value = roughness
    principled.inputs["Metallic"].default_value = metallic
    if color[3] < 1.0:
        principled.inputs["Alpha"].default_value = color[3]
        material.surface_render_method = "DITHERED"
    return material


def _mesh_object(name, positions, faces, material, matrix_world=None):
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    mesh.from_pydata([tuple(position) for position in positions], [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    if matrix_world is not None:
        obj.matrix_world = matrix_world
    return obj


def _overlay_object(name, buffers, material):
    positions = buffers["tri_positions"]
    faces = [tuple(range(index, index + 3)) for index in range(0, len(positions), 3)]
    return _mesh_object(name, positions, faces, material)


def _curve_object(name, line_positions, material, bevel_depth=0.035):
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = bevel_depth
    curve.bevel_resolution = 3
    for index in range(0, len(line_positions), 2):
        spline = curve.splines.new("POLY")
        spline.points.add(1)
        for point, position in zip(spline.points, line_positions[index:index + 2]):
            point.co = (*position, 1.0)
    obj = bpy.data.objects.new(name, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def _text_object(body, location, color_material, size=0.58):
    curve = bpy.data.curves.new(body, "FONT")
    curve.body = body
    curve.align_x = "CENTER"
    curve.size = size
    curve.extrude = 0.012
    obj = bpy.data.objects.new(body, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = location
    obj.data.materials.append(color_material)
    return obj


def _look_at(obj, target):
    direction = Vector(target) - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def _max_position_error(actual, expected):
    """Hausdorff-like ошибка множеств: triangulation меняет порядок углов."""

    actual_vectors = [Vector(position) for position in actual]
    expected_vectors = [Vector(position) for position in expected]
    forward = max(
        min((actual - expected).length for expected in expected_vectors)
        for actual in actual_vectors
    )
    backward = max(
        min((expected - actual).length for actual in actual_vectors)
        for expected in expected_vectors
    )
    return max(forward, backward)


def main():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    gray = _material("Source", (0.15, 0.18, 0.22, 1.0), metallic=0.1)
    red = _material("BeforeLocal", (0.95, 0.04, 0.025, 0.92), metallic=0.05)
    blue = _material("AfterWorld", (0.02, 0.28, 1.0, 0.92), metallic=0.12)
    white = _material("Labels", (0.92, 0.95, 1.0, 1.0))

    source_positions = (
        (-1.8, -1.0, 0.0),
        (1.8, -1.0, 0.0),
        (1.8, 1.0, 0.0),
        (-1.8, 1.0, 0.0),
    )
    preview_positions = tuple((x, y, 0.035) for x, y, _z in source_positions)
    face = DecalGeometryFace(
        surface_id=0,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("mw", index) for index in range(4)],
        positions=[Vector(position) for position in preview_positions],
        u_fracs=[-1.0, 1.0, 1.0, -1.0],
        v_lengths=[0.0, 0.0, 1.0, 1.0],
        component_kind="SEGMENT",
        component_side="acceptance",
    )

    basis = Matrix.LocRotScale(
        Vector((0.0, 0.0, 0.0)),
        Euler((math.radians(12.0), math.radians(-18.0), math.radians(27.0))),
        Vector((1.35, 0.72, 1.0)),
    )
    before_matrix = Matrix.Translation(Vector((-4.2, 0.0, 0.5))) @ basis
    after_matrix = Matrix.Translation(Vector((4.2, 0.0, 0.5))) @ basis

    before_source = _mesh_object(
        "Source_BEFORE_unapplied",
        source_positions,
        ((0, 1, 2, 3),),
        gray,
        before_matrix,
    )
    after_source = _mesh_object(
        "Source_AFTER_unapplied",
        source_positions,
        ((0, 1, 2, 3),),
        gray,
        after_matrix,
    )
    assert tuple(before_source.scale) != (1.0, 1.0, 1.0)
    assert tuple(after_source.scale) != (1.0, 1.0, 1.0)

    before_buffers = build_preview_buffers((face,))
    after_buffers = build_preview_buffers((face,), matrix_world=after_source.matrix_world)
    _overlay_object("BEFORE_local_space_overlay", before_buffers, red)
    _curve_object("BEFORE_local_space_outline", before_buffers["line_positions"], red)
    _overlay_object("AFTER_world_space_overlay", after_buffers, blue)
    _curve_object("AFTER_world_space_outline", after_buffers["line_positions"], blue)

    before_expected = [
        tuple(before_source.matrix_world @ Vector(position))
        for position in preview_positions
    ]
    after_expected = [
        tuple(after_source.matrix_world @ Vector(position))
        for position in preview_positions
    ]
    before_actual = list(dict.fromkeys(before_buffers["tri_positions"]))
    after_actual = list(dict.fromkeys(after_buffers["tri_positions"]))
    before_error = _max_position_error(before_actual, before_expected)
    after_error = _max_position_error(after_actual, after_expected)

    _text_object("BEFORE  local-space mismatch", (-4.0, 3.0, 0.0), red, 0.48)
    _text_object("AFTER  matrix_world aligned", (4.0, 3.0, 0.0), blue, 0.48)
    _text_object("red overlay remains at object-local origin", (0.0, -3.0, 0.0), white, 0.42)

    camera_data = bpy.data.cameras.new("AcceptanceCamera")
    camera = bpy.data.objects.new("AcceptanceCamera", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = (0.0, -13.8, 14.0)
    _look_at(camera, (0.0, 0.0, 0.0))
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 15.5
    bpy.context.scene.camera = camera

    key_data = bpy.data.lights.new("Key", "AREA")
    key_data.energy = 1400.0
    key_data.shape = "DISK"
    key_data.size = 8.0
    key = bpy.data.objects.new("Key", key_data)
    bpy.context.scene.collection.objects.link(key)
    key.location = (0.0, -2.0, 10.0)

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(OUTPUT_PNG)
    scene.render.film_transparent = False
    scene.world.color = (0.012, 0.015, 0.025)
    scene.view_settings.look = "AgX - Medium High Contrast"
    bpy.ops.render.render(write_still=True)

    result = {
        "blender_version": bpy.app.version_string,
        "source_transform_applied": False,
        "translation_rotation_non_uniform_scale": True,
        "before_local_space_max_error": before_error,
        "after_matrix_world_max_error": after_error,
        # mathutils Matrix хранит float32; это проверка точности записи, не
        # геометрический selector/порог rail-канона.
        "after_pass": after_error <= 1.0e-6,
        "image": str(OUTPUT_PNG),
    }
    OUTPUT_JSON.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("CFTUV_GPU_MATRIX_WORLD_ACCEPTANCE=" + json.dumps(result, sort_keys=True))
    if not result["after_pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
