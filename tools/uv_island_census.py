"""Перепись правила Straight strips на сцене: одна pChain - один непрерывный UV-остров (`UV_DIRECT_STRIP_V1`), по ширинам. Храповик падает по имени.

Headless на замороженной сцене, сцену не сохраняет (`-b scene.blend --python tools/uv_island_census.py -- out.json`).
`PYTHONPATH` выбирает код; `CFTUV_ISLAND_ALPHAS` - ширины через запятую. Выделение - все швы меша либо рёбра границы названного патча.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import time

import bmesh
import bpy

# SNAPSHOT_BYTE_CHECK_SAMPLED_V1: перепись проверяет каждый домен целиком.
os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"

#: Ширины и меши переписи. Меш отсутствует - отказ по имени, охват не сужается молча.
ALPHAS = (0.05, 0.1, 0.173, 0.3, 0.5, 1.0)
#: `(меш, патч)`: `None` - все швы меша, число - граница этого патча (полевая беда `rounded_wall_noise_top`, верхняя грань).
CASES = (("building", None), ("building.002", None), ("building.004", None), ("rounded_wall.001", None), ("rounded_wall_noise_top", 2))
#: Храповик: нарушения правила (pChain не с одним островом) - ноль; острова-веера между разными pChain - решение владельца, число не растёт.
UV_ISLAND_VIOLATIONS_ABOVE_RATCHET = "UV_ISLAND_VIOLATIONS_ABOVE_RATCHET"
UV_ISLAND_WEDGES_ABOVE_RATCHET = "UV_ISLAND_WEDGES_ABOVE_RATCHET"
SWEPT_MESH_ABSENT = "SWEPT_MESH_ABSENT"
RATCHET_FILE = Path(__file__).resolve().parents[1] / "artifacts" / "uv_islands_20261104" / "ratchet.json"


def _census(obj, patch, controller, alphas) -> list:
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    import cftuv.envelope_production_export as production
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget
    from cftuv_envelope.uv_islands import batch_island_census, pchains_of_uses

    settings = bpy.context.scene.hotspotuv_settings
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(obj.data)
    faces = tuple(face.index for face in bm.faces)
    revision = source_revision_from_bmesh(bm, obj, faces)
    keys = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(*keys, revision, lambda: build_analysis_bundle(bm, faces, obj))
    if patch is None:
        edges = frozenset(edge.index for edge in bm.edges if edge.seam)
    else:
        edges = frozenset(edge for loop in bundle.patch_graph.nodes[patch].boundary_loops for chain in loop.chains for edge in chain.edge_indices)
    rows = []
    for alpha in alphas:
        started = time.perf_counter()
        run = production.run_production(
            controller, bundle, edges, float(alpha), source_object_key=keys[0], source_data_key=keys[1], density=settings.envelope_debug_fan_density,
            developable_stretch_budget=envelope_stretch_budget(settings.envelope_debug_max_stretch),
            silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
            workers=0, kernel_backend="NATIVE", uv_policy_id="UV_DIRECT_STRIP_V1",
        )
        named = {}
        for prepared in controller._conveyor_preparation_cache.values():
            if getattr(prepared, "context", None) is not None:
                named.update(pchains_of_uses(prepared.context))
        row = {"mesh": obj.name, "alpha": alpha, "seconds": round(time.perf_counter() - started, 2), "domains": 0, "refused": 0, "pchains": 0, "satisfied": 0, "violations": [], "wedges": 0, "seam_edges": 0, "internal_seam_edges": 0, "islands": {}}
        for item in run.results:
            batch = getattr(item, "batch", None)
            if batch is None:
                row["refused"] += 1
                continue
            census = batch_island_census(batch, named)
            row["domains"] += 1
            row["pchains"] += len(census.pchains)
            row["satisfied"] += len(census.satisfied)
            row["violations"].extend((item.patch_id, pchain, count) for pchain, count in census.violations)
            row["wedges"] += len(census.wedges)
            row["seam_edges"] += census.seam_edges
            row["internal_seam_edges"] += census.internal_seam_edges
            row["islands"][str(item.patch_id)] = [[len(island.faces), *island.pchains] for island in census.islands]
        rows.append(row)
        print(f"ISLANDS {obj.name:<18} a={alpha:<6} {row['satisfied']}/{row['pchains']} pchains, wedges {row['wedges']}, seams {row['seam_edges']} (internal {row['internal_seam_edges']}), refused {row['refused']}, {row['seconds']} s", flush=True)
    bpy.ops.object.mode_set(mode="OBJECT")
    return rows


def main() -> None:
    import cftuv

    if not hasattr(bpy.types.Scene, "hotspotuv_settings"):
        cftuv.register()
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    output = Path(sys.argv[sys.argv.index("--") + 1])
    alphas = tuple(float(item) for item in os.environ.get("CFTUV_ISLAND_ALPHAS", "").split(",") if item) or ALPHAS
    controller = EnvelopeDebugSessionController()
    rows = []
    for name, patch in CASES:
        obj = bpy.data.objects.get(name)
        if obj is None or obj.type != "MESH":
            raise RuntimeError(f"{SWEPT_MESH_ABSENT}: {name}")
        rows.extend(_census(obj, patch, controller, alphas))
    ratchet = json.loads(RATCHET_FILE.read_text(encoding="utf-8")) if RATCHET_FILE.is_file() else {}
    violations = [f"{row['mesh']}@{row['alpha']}" for row in rows if len(row["violations"]) > ratchet.get("violations", 0)]
    wedges = [f"{row['mesh']}@{row['alpha']}" for row in rows if row["wedges"] > ratchet.get("wedges", {}).get(row["mesh"], 0)] if ratchet else []
    failure = UV_ISLAND_VIOLATIONS_ABOVE_RATCHET if violations else UV_ISLAND_WEDGES_ABOVE_RATCHET if wedges else None
    output.write_text(json.dumps({"rows": rows, "status": "REJECTED" if failure else "COMPLETE", "gate_failure": failure}, indent=1) + "\n", encoding="utf-8")
    if failure:
        raise RuntimeError(f"{failure}: {violations or wedges}")


if __name__ == "__main__":
    main()
