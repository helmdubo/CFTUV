"""Blender smoke для A9 world/local decal units contract.

Run:
    blender --background --factory-startup --python \
        artifacts/verify_decal_transform_units.py
"""

from __future__ import annotations

from pathlib import Path
import sys

import bpy
from mathutils import Matrix


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cftuv import decals as decals_module  # noqa: E402
from cftuv.decal_transform import (  # noqa: E402
    DecalSourceTransformError,
    local_decal_settings_for_source,
    validate_decal_source_transform,
)
from cftuv.model import DecalSettings, PatchGraph  # noqa: E402


def _source(name):
    mesh = bpy.data.meshes.new(name + "_Geo")
    mesh.from_pydata(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        (),
        ((0, 1, 2),),
    )
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _fill_metric_probe(bm, _graph, settings, *_args, **_kwargs):
    _fill_metric_probe.settings = settings
    verts = (
        bm.verts.new((0.0, 0.0, settings.offset)),
        bm.verts.new((1.0, 0.0, settings.offset)),
        bm.verts.new((1.0, settings.width_seam, settings.offset)),
        bm.verts.new((0.0, settings.width_seam, settings.offset)),
    )
    face = bm.faces.new(verts)
    uv_layer = bm.loops.layers.uv.new("UVMap")
    for loop in face.loops:
        loop[uv_layer].uv = (
            loop.vert.co.y,
            loop.vert.co.x * settings.uv_length_scale,
        )
    return ()


def _remove_object(obj):
    if obj is None:
        return
    mesh = obj.data if obj.type == "MESH" else None
    bpy.data.objects.remove(obj, do_unlink=True)
    if mesh is not None and mesh.users == 0:
        bpy.data.meshes.remove(mesh)


def main():
    source = _source("A9_Source")
    source.scale = (2.0, 2.0, 2.0)
    bpy.context.view_layer.update()
    world_settings = DecalSettings(
        width_seam=0.15,
        offset=0.02,
        uv_length_scale=0.25,
    )
    original_fill = decals_module._fill_decal_bmesh
    decal = None
    try:
        decals_module._fill_decal_bmesh = _fill_metric_probe
        names = decals_module.generate_decal_objects(
            PatchGraph(),
            source,
            world_settings,
            "SEAMS",
            scene=bpy.context.scene,
        )
        decal = bpy.data.objects[names[0]]
        local = _fill_metric_probe.settings
        world_points = [decal.matrix_world @ vert.co for vert in decal.data.vertices]
        world_width = max(point.y for point in world_points) - min(
            point.y for point in world_points
        )
        world_offset = min(point.z for point in world_points)
        world_length = max(point.x for point in world_points) - min(
            point.x for point in world_points
        )
        uv_values = [tuple(loop.uv) for loop in decal.data.uv_layers.active.data]
        uv_span = max(value[1] for value in uv_values) - min(
            value[1] for value in uv_values
        )

        assert local.width_seam == 0.075
        assert local.offset == 0.01
        assert local.uv_length_scale == 0.5
        assert abs(world_width - world_settings.width_seam) <= 1e-7
        assert abs(world_offset - world_settings.offset) <= 1e-7
        assert abs(world_length - 2.0) <= 1e-7
        assert abs(
            uv_span - world_length * world_settings.uv_length_scale
        ) <= 1e-7

        rotated = _source("A9_Rotated")
        rotated.matrix_world = (
            Matrix.Translation((3.0, -2.0, 5.0))
            @ Matrix.Rotation(0.75, 4, "Z")
            @ Matrix.Scale(2.0, 4)
        )
        rotated_local = local_decal_settings_for_source(
            world_settings, rotated
        )
        assert abs(rotated_local.width_seam - local.width_seam) <= 1e-8
        assert abs(rotated_local.offset - local.offset) <= 1e-8
        assert abs(
            rotated_local.uv_length_scale - local.uv_length_scale
        ) <= 1e-8
        _remove_object(rotated)

        rejected = {}
        for label, matrix in (
            ("non_uniform", Matrix.Diagonal((2.0, 1.0, 1.0, 1.0))),
            (
                "shear",
                Matrix(
                    (
                        (1.0, 0.25, 0.0, 0.0),
                        (0.0, 1.0, 0.0, 0.0),
                        (0.0, 0.0, 1.0, 0.0),
                        (0.0, 0.0, 0.0, 1.0),
                    )
                ),
            ),
            ("mirror", Matrix.Diagonal((-1.0, 1.0, 1.0, 1.0))),
        ):
            try:
                validate_decal_source_transform(matrix)
            except DecalSourceTransformError as exc:
                rejected[label] = exc.code
        assert rejected == {
            "non_uniform": "NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM",
            "shear": "NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM",
            "mirror": "MIRRORED_SOURCE_TRANSFORM",
        }
        print(
            "[CFTUV][A9] PASS",
            f"world_width={world_width:.6f}",
            f"world_offset={world_offset:.6f}",
            f"uv_span={uv_span:.6f}",
            rejected,
        )
    finally:
        decals_module._fill_decal_bmesh = original_fill
        _remove_object(decal)
        _remove_object(source)


main()
