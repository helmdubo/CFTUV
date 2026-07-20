"""Blender smoke для A7 temporary preview lifecycle.

Run:
    blender --background --factory-startup --python \
        artifacts/verify_decal_preview_lifecycle.py
"""

from __future__ import annotations

from pathlib import Path
import sys

import bmesh
import bpy


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cftuv.decals import (  # noqa: E402
    DecalPreviewState,
    _decal_object_name,
    _decal_preview_object_name,
    _finalize_decal_object,
    _set_decal_preview_metadata,
    remove_decal_preview_object,
)
from cftuv import decals as decals_module  # noqa: E402


def _triangle_bmesh(scale=1.0):
    bm = bmesh.new()
    verts = (
        bm.verts.new((0.0, 0.0, 0.0)),
        bm.verts.new((scale, 0.0, 0.0)),
        bm.verts.new((0.0, scale, 0.0)),
    )
    bm.faces.new(verts)
    return bm


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


def _fill_transaction_triangle(bm, *_args, **_kwargs):
    verts = (
        bm.verts.new((0.0, 0.0, 0.0)),
        bm.verts.new((1.0, 0.0, 0.0)),
        bm.verts.new((0.0, 1.0, 0.0)),
    )
    bm.faces.new(verts)
    return ()


def main():
    scene = bpy.context.scene
    source = _source("A7_Source")
    production_name = _decal_object_name("SEAMS", source)
    production_mesh = bpy.data.meshes.new(production_name + "_OldGeo")
    production_mesh.from_pydata(
        ((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (0.0, 0.5, 0.0)),
        (),
        ((0, 1, 2),),
    )
    material = bpy.data.materials.new("A7_Material")
    production_mesh.materials.append(material)
    production = bpy.data.objects.new(production_name, production_mesh)
    scene.collection.objects.link(production)
    production["artist"] = "preserved"
    production_pointer = production.as_pointer()
    old_mesh_pointer = production_mesh.as_pointer()
    old_mesh_name = production_mesh.name

    preview_state = DecalPreviewState()
    preview_name = _decal_preview_object_name("SEAMS", source)
    preview = _finalize_decal_object(
        _triangle_bmesh(1.0),
        preview_name,
        source,
        scene,
        reuse_existing=True,
        preview_state=preview_state,
        prepared=True,
    )
    _set_decal_preview_metadata(preview, source)
    preview_pointer = preview.as_pointer()
    preview_mesh_pointer = preview.data.as_pointer()
    assert production.as_pointer() == production_pointer
    assert production.data.as_pointer() == old_mesh_pointer
    assert preview.hide_render is True
    assert preview["cftuv_decal_preview"] is True

    preview = _finalize_decal_object(
        _triangle_bmesh(1.25),
        preview_state.object_name,
        source,
        scene,
        reuse_existing=True,
        preview_state=preview_state,
        prepared=True,
    )
    _set_decal_preview_metadata(preview, source)
    assert preview.as_pointer() == preview_pointer
    assert preview.data.as_pointer() == preview_mesh_pointer

    assert remove_decal_preview_object("SEAMS", source, preview_state) is True
    assert bpy.data.objects.get(preview_name) is None
    assert production.as_pointer() == production_pointer
    assert production.data.as_pointer() == old_mesh_pointer
    assert production["artist"] == "preserved"

    confirmed = _finalize_decal_object(
        _triangle_bmesh(2.0),
        production_name,
        source,
        scene,
        prepared=True,
    )
    assert confirmed is production
    assert production.as_pointer() == production_pointer
    assert production.data.as_pointer() != old_mesh_pointer
    assert production["artist"] == "preserved"
    assert tuple(production.data.materials) == (material,)
    assert old_mesh_name not in bpy.data.meshes

    source_without_final = _source("A7_NoProduction")
    empty_state = DecalPreviewState()
    empty_preview_name = _decal_preview_object_name(
        "SEAMS", source_without_final
    )
    empty_preview = _finalize_decal_object(
        _triangle_bmesh(0.75),
        empty_preview_name,
        source_without_final,
        scene,
        reuse_existing=True,
        preview_state=empty_state,
        prepared=True,
    )
    _set_decal_preview_metadata(empty_preview, source_without_final)
    assert bpy.data.objects.get(
        _decal_object_name("SEAMS", source_without_final)
    ) is None
    assert remove_decal_preview_object(
        "SEAMS", source_without_final, empty_state
    ) is True

    # Реальный Apply-order: production transaction выполняется до terminal
    # cleanup и не должна переписать identity временного object.
    apply_source = _source("A7_ApplyFlow")
    apply_state = DecalPreviewState()
    original_fill = decals_module._fill_decal_bmesh
    decals_module._fill_decal_bmesh = _fill_transaction_triangle
    try:
        preview_result = decals_module._generate_decal_transaction(
            None,
            apply_source,
            None,
            "SEAMS",
            scene,
            None,
            None,
            True,
            None,
            apply_state,
        )
        apply_preview_name = preview_result.obj.name
        assert apply_state.object_name == apply_preview_name
        final_result = decals_module._generate_decal_transaction(
            None,
            apply_source,
            None,
            "SEAMS",
            scene,
            None,
            None,
            False,
            None,
            apply_state,
        )
        assert final_result.obj.name == _decal_object_name(
            "SEAMS", apply_source
        )
        assert apply_state.object_name == apply_preview_name
        assert remove_decal_preview_object(
            "SEAMS", apply_source, apply_state
        ) is True
        assert bpy.data.objects.get(apply_preview_name) is None
        assert bpy.data.objects.get(final_result.obj.name) is final_result.obj
    finally:
        decals_module._fill_decal_bmesh = original_fill

    generated = bpy.data.collections.get("Decals_Generated")
    assert generated is not None
    assert not any(
        obj.get("cftuv_decal_preview", False) for obj in generated.objects
    )
    assert not any(
        mesh.name.startswith(".CFTUV_Preview") for mesh in bpy.data.meshes
    )
    print(
        "[CFTUV][A7] PASS",
        production.name,
        production.as_pointer(),
        production.data.as_pointer(),
        tuple(slot.name for slot in production.data.materials),
    )


main()
