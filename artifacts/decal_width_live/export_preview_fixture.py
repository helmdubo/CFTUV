"""Выгрузка входов превью ширины декали с настоящего меша сцены в JSON-фикстуру (DECAL-WIDTH-LIVE).

    blender -b E:\\testscene.blend --python-exit-code 1 --python export_preview_fixture.py -- \\
        --root <дерево> --mesh rounded_wall.001 --out <json> [--workers 0] [--density 2]

Выделяются ВСЕ швы меша (как перед «Build Decal Mesh»), кнопка исполняется настоящим оператором, а
`build_preview_inputs` подслушивается: в файл идут ровно `PatchSurfaceIR` (вершины, рёбра с гранями, грани с
циклами и нормалями) и `selected_by_patch`, из которых строится превью. Тест
`tests/test_envelope_width_preview.py` читает этот файл без Blender (`load_preview_fixture`).
Аддон берётся ИЗ ДЕРЕВА `--root`; файл сцены не сохраняется.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import bmesh
import bpy


def _arguments() -> argparse.Namespace:
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--density", default="2")
    return parser.parse_args(tail)


def _load_tree(root: Path) -> None:
    installed = sys.modules.get("cftuv")
    if installed is not None:
        installed.unregister()
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        if str(path) in sys.path:
            sys.path.remove(str(path))
        sys.path.insert(0, str(path))
    import cftuv

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    cftuv.register()


def _select_seams(obj) -> int:
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
    wanted = 0
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
        wanted += int(bool(edge.seam))
    bmesh.update_edit_mesh(obj.data)
    return wanted


def main() -> None:
    args = _arguments()
    _load_tree(Path(args.root).resolve())
    from cftuv import envelope_width_live as live
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    captured: dict = {}
    original = live.build_preview_inputs

    def spy(surface, selected_by_patch):
        captured["surface"], captured["selected"] = surface, selected_by_patch
        return original(surface, selected_by_patch)

    live.build_preview_inputs = spy
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = args.workers
    settings.envelope_debug_fan_density = args.density
    controller = getattr(bpy.context.window_manager, "_cftuv_envelope_debug_session", None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        bpy.context.window_manager._cftuv_envelope_debug_session = controller
    controller.clear()
    _select_seams(bpy.data.objects[args.mesh])
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
    surface = captured["surface"]
    payload = {
        "mesh": args.mesh,
        "selected_by_patch": [[int(patch), [int(edge) for edge in edges]] for patch, edges in captured["selected"]],
        "vertices": [[int(item.vertex_id), [float(c) for c in item.position]] for item in surface.vertices],
        "edges": [
            [int(item.edge_id), [int(v) for v in item.vertex_ids], [int(f) for f in item.source_face_ids]]
            for item in surface.edges
        ],
        "faces": [
            {
                "face_id": int(item.face_id),
                "patch_id": int(item.patch_id),
                "vertex_cycle": [int(v) for v in item.vertex_cycle],
                "edge_cycle": [int(e) for e in item.edge_cycle],
                "polygon_normal": [float(c) for c in item.polygon_normal],
            }
            for item in surface.faces
        ],
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("FIXTURE_EXPORTED", out)


main()
