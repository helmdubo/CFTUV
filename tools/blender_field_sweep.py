"""Свип приёмки: НАСТОЯЩИЙ оператор Build Envelope Debug на каждом меше сцены.

Зачем он существует, честно: полевой гейт гоняет ядро НАПРЯМУЮ и обходит
операторный код — выбор цепочек, охранники выделения, GP-рендер. Владелец
дважды нашёл поломки именно там, куда гейт не смотрит. Этот скрипт жмёт ту же
кнопку, что владелец (`hotspotuv.build_exact_reference_envelope_debug`), на
КАЖДОМ меше сцены с полным выделением швов, плюс сценарий «одно ребро», и
печатает именованные исходы. Сцену не сохраняет.

Запуск: blender -b сцена --python-exit-code 1 --python этот_файл [-- вывод.json]
"""

from __future__ import annotations

import json
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for entry in (str(ROOT), str(ROOT / "kernel" / "src")):
    if entry in sys.path:
        sys.path.remove(entry)
    sys.path.insert(0, entry)
for module_name in tuple(sys.modules):
    if module_name.split(".", 1)[0] in ("cftuv", "cftuv_envelope"):
        del sys.modules[module_name]

import bmesh
import bpy

import cftuv

cftuv.register()


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _select_seams(obj, edge_indices=None):
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.objects.active = obj
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select_set(False)
    wanted = 0
    for edge in bm.edges:
        seam = edge.seam
        if edge_indices is not None:
            seam = edge.index in edge_indices
        if seam:
            edge.select_set(True)
            wanted += 1
    bmesh.update_edit_mesh(obj.data)
    return wanted


def _run_operator(obj, label, edge_indices=None):
    report = {"mesh": obj.name, "scenario": label}
    try:
        selected = _select_seams(obj, edge_indices)
        report["selected_edges"] = selected
        if not selected:
            report["result"] = "SKIP_NO_SELECTION"
            return report
        settings = _settings()
        settings.envelope_debug_engine = "QUEUE"
        outcome = bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        report["result"] = sorted(outcome)
        for key in (
            "envelope_debug_status",
            "envelope_debug_outcome",
            "envelope_debug_domain_status",
            "envelope_debug_stage_summary",
            "envelope_debug_queue_timing",
        ):
            report[key] = getattr(settings, key, "")
    except Exception:
        report["result"] = "EXCEPTION"
        report["traceback"] = traceback.format_exc(limit=6)
    finally:
        try:
            bpy.ops.object.mode_set(mode="OBJECT")
        except Exception:
            pass
    return report


def main() -> None:
    arguments = (
        sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    )
    output = Path(arguments[0]) if arguments else None

    reports = []
    meshes = [
        obj
        for obj in bpy.data.objects
        if obj.type == "MESH"
        and any(edge.use_seam for edge in obj.data.edges)
    ]
    for obj in meshes:
        reports.append(_run_operator(obj, "all_seams"))

    # Сценарий владельца: ОДНО шовное ребро — envelope обязан встать по обе
    # стороны (оба патча-соседа). Берётся первое шовное ребро каждого меша.
    for obj in meshes:
        first_seam = next(
            (edge.index for edge in obj.data.edges if edge.use_seam), None
        )
        if first_seam is not None:
            reports.append(
                _run_operator(obj, "single_edge", edge_indices={first_seam})
            )

    payload = {"scene": bpy.data.filepath, "reports": reports}
    text = json.dumps(payload, ensure_ascii=False, indent=1)
    if output is not None:
        output.write_text(text + "\n", encoding="utf-8")
    for report in reports:
        status = report.get("envelope_debug_status", "")
        summary = report.get("envelope_debug_stage_summary", "")
        print(
            f"SWEEP {report['mesh']:<28} {report['scenario']:<12} "
            f"{report.get('result')} | {status[:80]} | {summary[:80]}"
        )
        if report.get("traceback"):
            print(report["traceback"].splitlines()[-1])
    print("SWEEP_DONE", len(reports))


if __name__ == "__main__":
    main()
