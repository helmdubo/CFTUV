"""НАСТОЯЩАЯ кнопка QUEUE (debug-оператор) на мешах сцены: суммы вееров из сайдкара против перечня.

  blender.exe -b E:\\testscene.blend --python artifacts/fan_consistency/blender_button_check.py -- <results_dir> <mesh[;mesh..]> <densities>

Аддон и ядро берутся ИЗ ЭТОГО дерева (sys.path, модули cftuv* выброшены из кэша). Для каждого меша и плотности:
выделяются все швы, ставятся engine=QUEUE и Fan Density, зовётся
`bpy.ops.hotspotuv.build_exact_reference_envelope_debug()`, читается сайдкар (Text datablock) и суммируются
счётчики `CONVEYOR_RATIONAL_VERTEX_FANS` / `CONVEYOR_FAN_SUPPORTS` по доменам. Печатается сравнение с перечнем
`fans_<mesh>_d<d>.json` (сумма spec_H). Сцену НЕ сохраняет.
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

import cftuv  # noqa: E402

assert Path(cftuv.__file__).resolve().parents[1] == ROOT, cftuv.__file__
from cftuv.envelope_debug_renderer import envelope_debug_text_name  # noqa: E402
from cftuv.operators import classes  # noqa: E402


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
    count = 0
    for edge in bm.edges:
        if edge.seam:
            edge.select_set(True)
            count += 1
    bmesh.update_edit_mesh(obj.data)
    return count


def _counter_sum(domains, name):
    total = 0
    for domain in domains:
        counters = domain.get("counters") or domain.get("preparation_counters") or {}
        if isinstance(counters, dict):
            total += counters.get(name, 0)
        else:
            total += sum(item["value"] for item in counters if item["name"] == name)
    return total


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1:]
    results_dir, meshes, densities = Path(argv[0]), argv[1].split(";"), [int(x) for x in argv[2].split(",")]
    registered = [c for c in classes if c.is_registered]
    if not registered:
        cftuv.register()
    elif len(registered) != len(classes):
        raise SystemExit("partial registration")
    settings = bpy.context.scene.hotspotuv_settings
    for mesh in meshes:
        obj = bpy.data.objects[mesh]
        for density in densities:
            selected = _select_seams(obj)
            settings.envelope_debug_engine = "QUEUE"
            settings.envelope_debug_fan_density = str(density)
            settings.envelope_debug_alpha = 0.25
            text_name = envelope_debug_text_name(obj)
            previous = bpy.data.texts.get(text_name)
            if previous is not None:
                bpy.data.texts.remove(previous)
            outcome = bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
            payload = json.loads(bpy.data.texts[text_name].as_string())
            domains = (payload.get("queue") or {}).get("domains", [])
            if mesh == meshes[0] and density == densities[0] and domains:
                print("BUTTON sidecar domain keys:", sorted(domains[0].keys()))
                print("BUTTON sidecar counters sample:", str(domains[0].get("counters"))[:600])
                print("BUTTON sidecar host_counters sample:", str(domains[0].get("host_counters"))[:300])
            fans = _counter_sum(domains, "CONVEYOR_RATIONAL_VERTEX_FANS")
            supports = _counter_sum(domains, "CONVEYOR_FAN_SUPPORTS")
            exact = sum(1 for d in domains if d.get("preparation_outcome") == "EXACT")
            enumerated = json.loads(
                (results_dir / f"fans_{mesh.replace('.', '_')}_d{density}.json").read_text(encoding="utf-8")
            )
            enum_fans = sum(len(r["corners"]) for r in enumerated)
            enum_supports = sum(c["spec_H"] for r in enumerated for c in r["corners"])
            verdict = "SAME" if (fans, supports) == (enum_fans, enum_supports) else "DIFF"
            print(
                f"BUTTON {mesh} d{density}: result={sorted(outcome)} seams={selected} domains={len(domains)} exact={exact} "
                f"button fans={fans} supports={supports} | enumerated fans={enum_fans} supports={enum_supports} {verdict}"
            )
    bpy.ops.object.mode_set(mode="OBJECT")


main()
