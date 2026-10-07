"""Выгрузка полевой фикстуры стены: вершины стен доменов `rounded_wall.001`, выделение = все цепи патча верхней крышки.

    blender -b E:\\testscene.blend --python-exit-code 1 --python artifacts/wall_miter/export_wall_fixture.py -- <выход.json> [alpha ...]

Выделение (как у владельца): все граничные цепи патча крышки (патч с нормалью `+z` и наибольшей высотой центра) — на `rounded_wall.001` это
четыре цепи, 30 рёбер. Соседние патчи (две длинные грани и два торца) становятся доменами, а вертикальные швы между ними НЕ выделены: это
стены, у которых сходятся две области. Маршрут — кнопка (`run_production`, плотность веера 2, допуск растяжения 42 %, один процесс).

Пишется ровно то, что читает сварка (`DomainVerticesV1`): по домену вершины стен (позиция батча, нормаль смещения, ссылка) и цепи стен в локальных
номерах. Числа — `repr` binary64 (точные); смещение — значение сцены. Тест: `tests/test_envelope_production_wall.py` (Blender не нужен).
Blender только headless, без сохранения .blend; аддон берётся из дерева репозитория (установленный снимается).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "cftuv.wall_miter_field_fixture.v1"
MESH = "rounded_wall.001"
DENSITY = "2"
STRETCH_PERCENT = 42
CAP_NORMAL_Z = 0.9


def _load_tree(root: Path) -> None:
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
            del sys.modules[name]
    tree = [str(root), str(root / "kernel" / "src")]
    sys.path[:] = [*tree, *(item for item in sys.path if item not in tree)]
    import cftuv

    cftuv.register()


def _restricted(view) -> dict:
    """Вершины домена, лежащие на цепях стен, и сами цепи в новых локальных номерах."""

    vertices = view.vertices
    used = sorted({local for chain in vertices.walls for local in chain})
    remap = {local: number for number, local in enumerate(used)}
    return {
        "patch_id": vertices.patch_id,
        "positions": [list(vertices.positions[local]) for local in used],
        "normals": [list(vertices.normals[local]) for local in used],
        "refs": [vertices.refs[local] for local in used],
        "walls": [[remap[local] for local in chain] for chain in vertices.walls],
    }


def main() -> None:
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    out = Path(arguments[0])
    alphas = [float(item) for item in arguments[1:]] or [0.2239]
    _load_tree(ROOT)
    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_production_export import run_production
    from cftuv.envelope_production_view import build_domain_view
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    obj = bpy.data.objects[MESH]
    settings = bpy.context.scene.hotspotuv_settings
    mesh_settings = bpy.context.scene.hotspotuv_decal_mesh
    bpy.context.view_layer.objects.active = obj
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in bm.faces)
    controller = EnvelopeDebugSessionController()
    setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    revision = source_revision_from_bmesh(bm, obj, face_indices)
    object_key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(object_key, data_key, revision, lambda: build_analysis_bundle(bm, face_indices, obj))
    cap = max((node for node in bundle.patch_graph.nodes.values() if node.normal.z > CAP_NORMAL_Z), key=lambda node: node.centroid.z)
    selected = sorted({edge for loop in cap.boundary_loops for chain in loop.chains for edge in chain.edge_indices})
    report = {
        "schema": SCHEMA,
        "mesh": MESH,
        "selection": f"every boundary chain of the top cap patch {cap.patch_id}: {len(selected)} edges",
        "selected_edges": selected,
        "density": DENSITY,
        "stretch_percent": STRETCH_PERCENT,
        "offset": float(mesh_settings.offset),
        "runs": [],
    }
    for alpha in alphas:
        run = run_production(
            controller,
            bundle,
            frozenset(selected),
            alpha,
            source_object_key=object_key,
            source_data_key=data_key,
            density=DENSITY,
            developable_stretch_budget=envelope_stretch_budget(STRETCH_PERCENT),
            silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
            workers=0,
        )
        domains = [
            _restricted(build_domain_view(result))
            for result in sorted(run.results, key=lambda item: (item.patch_id, item.domain_id))
            if result.is_materialized and any(name.split(":")[1] == "WALL" for name, _keys in build_domain_view(result).boundary_chains)
        ]
        report["runs"].append({"alpha": alpha, "domains": domains})
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    out.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print("WALL_FIXTURE_WRITTEN", out, [len(item["domains"]) for item in report["runs"]])


main()
