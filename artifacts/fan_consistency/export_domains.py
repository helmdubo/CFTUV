"""Выгрузка доменов меша(ей) из живой сцены (Blender, headless) для диагноза согласованности вееров.

Копия artifacts/plan_not_compiled/export_domains.py: несколько мешей за один сеанс,
маршрут не изменён.

Запуск (ИЗ ДЕРЕВА ЭТОЙ ВЕТКИ; сцену НЕ сохранять — скрипт её только читает):

  blender.exe -b E:\\testscene.blend --python artifacts/plan_not_compiled/export_domains.py \
      -- <out_dir> <mesh> <patch_ids|ALL> <densities> [alpha]

Маршрут тот же, что у кнопки QUEUE: все швы выделены -> `build_analysis_bundle`
по рёберной выборке -> `stage_domain_inputs` -> на каждый домен
`build_envelope_analysis_snapshot(included_patch_ids={patch})` и
`build_envelope_decal_request(density=d)`. Аддон и ядро берутся ИЗ ЭТОГО дерева
(sys.path, модули `cftuv*` предварительно выбрасываются из кэша — префы владельца
автовключают установленную копию).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for entry in (str(ROOT / "kernel" / "src"), str(ROOT)):
    if entry in sys.path:
        sys.path.remove(entry)
    sys.path.insert(0, entry)
for name in [n for n in sys.modules if n == "cftuv" or n.startswith("cftuv.") or n.startswith("cftuv_envelope")]:
    del sys.modules[name]

import bmesh  # noqa: E402
import bpy  # noqa: E402


def _args():
    argv = sys.argv
    return argv[argv.index("--") + 1:] if "--" in argv else []


def _select_seams(obj):
    bpy.context.view_layer.objects.active = obj
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select_set(False)
    for edge in bm.edges:
        if edge.seam:
            edge.select_set(True)
    bmesh.update_edit_mesh(obj.data)


def main():
    out_dir, mesh_names, patches_arg, densities_arg, *rest = _args()
    for mesh_name in mesh_names.split(';'):
        _one(out_dir, mesh_name, patches_arg, densities_arg, *rest)


def _one(out_dir, mesh_name, patches_arg, densities_arg, *rest):
    alpha = float(rest[0]) if rest else 0.25
    densities = [int(x) for x in densities_arg.split(",")]
    out = Path(out_dir) / mesh_name.replace(".", "_")
    out.mkdir(parents=True, exist_ok=True)

    import cftuv
    assert Path(cftuv.__file__).resolve().parents[1] == ROOT, cftuv.__file__
    import cftuv_envelope
    assert Path(cftuv_envelope.__file__).resolve().parents[2] == ROOT / "kernel", cftuv_envelope.__file__
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_request_export import (
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs
    from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1

    obj = bpy.data.objects.get(mesh_name)
    print("MESHES", [o.name for o in bpy.data.objects if o.type == "MESH"])
    if obj is None:
        raise SystemExit(f"mesh {mesh_name!r} absent")
    _select_seams(obj)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    selected = frozenset(edge.index for edge in bm.edges if edge.select)
    bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in bm.faces)
    bundle = build_analysis_bundle(bm, face_indices, obj)
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, selected)
    wanted = list(patch_ids) if patches_arg == "ALL" else [int(x) for x in patches_arg.split(",")]
    manifest = {
        "object_name": mesh_name,
        "alpha": str(alpha),
        "densities": densities,
        "selected_edges": len(selected),
        "patch_count": len(patch_ids),
        "request_id": request_id,
        "domains": {},
    }
    for patch_id in wanted:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle, included_patch_ids=frozenset({patch_id})
            )
        except Exception as exc:  # хост-отказ метрики (near-planar и т. п.) — не наш предмет
            manifest.setdefault("host_refused", {})[str(patch_id)] = type(exc).__name__ + ": " + str(exc)[:120]
            print("HOST_REFUSED", mesh_name, patch_id)
            continue
        sub = out / f"patch_{patch_id}"
        sub.mkdir(exist_ok=True)
        (sub / "analysis_snapshot.json").write_bytes(AnalysisSnapshotCodecV1.dumps(snapshot))
        for density in densities:
            request = build_envelope_decal_request(
                snapshot,
                frozenset(by_domain[domain_id]),
                alpha,
                decal_request_id_value=request_id,
                density=density,
            )
            (sub / f"decal_request_d{density}.json").write_bytes(DecalRequestCodecV1.dumps(request))
        manifest["domains"][str(patch_id)] = domain_id
        print("EXPORTED", mesh_name, patch_id, domain_id)
    (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")
    bpy.ops.object.mode_set(mode="OBJECT")


main()
