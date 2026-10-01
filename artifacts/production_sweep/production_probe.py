"""Настоящая кнопка «Build Decal Mesh» на мешах сцены в фоновом Blender.

Аддон берётся ИЗ ДЕРЕВА `--root` (как в `warm_coverage_parallel/button_probe.py`):
установленная копия снимается с регистрации, пути дерева и его `kernel/src` встают
первыми. Выделение — все швы меша (`edge.seam`), Fan Density — `--density`.

    blender -b E:\\testscene.blend --python-exit-code 1 --python production_probe.py -- \\
        --root <дерево> --out <json> [--workers 8] [--mesh building.002] \\
        [--density 2] [--steps cold,rebuild,debug_warm] [--obj <файл.obj>]

Шаги:

* `cold` — сессия пуста, нажатие продукта БЕЗ предшествующей кнопки отладки
  (холодное: наполняет кэши отладочным вычислителем, затем продукт);
* `rebuild` — второе нажатие на той же сессии: пересборка объекта, дайджест тот же;
* `debug_warm` — сессия сброшена, кнопка отладки (QUEUE), затем продукт: тёплое
  нажатие, счётчики сборок контроллера обязаны остаться на месте.

После каждого шага пишется отпечаток объекта (вершины, грани, петли, слой UV,
дайджест того, что лежит в Blender), исходы по доменам, секунды и сборки.
Всё, что не секунды, обязано совпадать между `--workers 0` и `--workers 8`
(`compare_probe.py`).

`--export-json <каталог>` — после первого шага батчи MATERIALIZED-доменов
кодеком ядра (`GeometryBatchCodecV1`) и сводка исходов (`export_production_json`).

`--obj` — после первого шага мех пишется в OBJ (локальные координаты источника,
`vt` из слоя UV, 5 знаков) для последующего осмотра глазами.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy


def _arguments() -> argparse.Namespace:
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--mesh", default="building.002")
    parser.add_argument("--density", default="2")
    parser.add_argument("--alpha", type=float, default=0.25)
    parser.add_argument("--steps", default="cold,rebuild,debug_warm")
    parser.add_argument("--obj", default="")
    parser.add_argument("--export-json", default="")
    return parser.parse_args(tail)


def _load_tree(root: Path):
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(
            ("cftuv.", "cftuv_envelope.")
        ):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert (
        Path(cftuv_envelope.__file__).resolve().parent
        == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    )
    cftuv.register()
    return cftuv


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


CAPTURED: dict = {}


def _install_capture() -> None:
    """Запоминает прогон и квитанцию, не меняя ни того, ни другого."""

    from cftuv import envelope_production_export, envelope_production_operator

    original_run = envelope_production_export.run_production

    def run_production(*args, **kwargs):
        run = original_run(*args, **kwargs)
        CAPTURED["run"] = run
        return run

    envelope_production_export.run_production = run_production
    original_write = envelope_production_operator.write_decal_object

    def write_decal_object(*args, **kwargs):
        receipt = original_write(*args, **kwargs)
        CAPTURED["receipt"] = receipt
        return receipt

    envelope_production_operator.write_decal_object = write_decal_object


def _controller():
    return getattr(bpy.context.window_manager, "_cftuv_envelope_debug_session", None)


def _builds() -> dict:
    controller = _controller()
    return {} if controller is None else controller.build_counts


def _reset_session() -> None:
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    controller = _controller()
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        bpy.context.window_manager._cftuv_envelope_debug_session = controller
    controller.clear()


def _object_stats(source_name: str) -> dict:
    from cftuv.envelope_production_mesh import decal_object_name, mesh_content_digest

    decal = bpy.data.objects.get(decal_object_name(source_name))
    if decal is None:
        return {"object": None}
    mesh = decal.data
    layer = mesh.uv_layers.get("UVMap")
    return {
        "object": decal.name,
        "parent": None if decal.parent is None else decal.parent.name,
        "vertices": len(mesh.vertices),
        "faces": len(mesh.polygons),
        "loops": len(mesh.loops),
        "uv_layer": None if layer is None else layer.name,
        "uv_loops": 0 if layer is None else len(layer.data),
        "seam_edges": sum(1 for edge in mesh.edges if edge.use_seam),
        "attributes": sorted(
            item.name for item in mesh.attributes if item.name.startswith("cftuv_")
        ),
        "materials": [item.name for item in mesh.materials if item is not None],
        "mesh_digest": mesh_content_digest(mesh),
    }


def _run_stats() -> dict:
    run = CAPTURED.get("run")
    receipt = CAPTURED.get("receipt")
    if run is None:
        return {}
    outcomes: dict[str, int] = {}
    for item in run.results:
        outcomes[item.outcome] = outcomes.get(item.outcome, 0) + 1
    counters: dict[str, int] = {}
    for item in run.results:
        for name, value in item.counters:
            if name.startswith("MATERIALIZE_"):
                counters[name] = counters.get(name, 0) + int(value)
    diagnostics: dict[str, int] = {}
    for item in run.results:
        for line in item.diagnostics:
            name = line.split(":", 1)[0]
            diagnostics[name] = diagnostics.get(name, 0) + 1
    placements: dict[str, int] = {}
    for item in run.results:
        placements[item.placement.split(":")[0]] = (
            placements.get(item.placement.split(":")[0], 0) + 1
        )
    profile = {
        item.name: item.value
        for item in run.profile.counters
        if item.patch_domain_id is None
        and (item.name.startswith(("PRODUCTION_", "ENVELOPE_DOMAIN_POOL_")))
    }
    return {
        "domains": len(run.results),
        "outcomes": dict(sorted(outcomes.items())),
        "refused": [
            {
                "patch_id": item.patch_id,
                "outcome": item.outcome,
                "detail": item.detail[:160],
            }
            for item in run.results
            if not item.is_materialized
        ],
        "cold": run.cold,
        "run_seconds": round(run.wall_seconds, 3),
        "placements": dict(sorted(placements.items())),
        "materialize_counters": dict(sorted(counters.items())),
        "diagnostics": dict(sorted(diagnostics.items())),
        "chart_orientations": {
            str(item.patch_id): item.chart_orientation
            for item in run.results
            if item.chart_orientation
        },
        "chart_orientation_counts": dict(
            sorted(
                {
                    name: sum(1 for item in run.results if item.chart_orientation == name)
                    for name in {item.chart_orientation for item in run.results}
                    if name
                }.items()
            )
        ),
        "profile_counters": dict(sorted(profile.items())),
        "content_digests": {
            str(item.patch_id): item.content_digest for item in run.results
        },
        "receipt": None
        if receipt is None
        else {
            "arrays_digest": receipt.arrays_digest,
            "mesh_digest": receipt.mesh_digest,
            "mesh_name": receipt.mesh_name,
            "object_name": receipt.object_name,
            "replaced": receipt.replaced,
            "seam_edges": receipt.seam_edges,
            "seam_edges_requested": receipt.seam_edges_requested,
            "skipped": [list(item) for item in receipt.skipped],
            "warnings": [list(item) for item in receipt.warnings],
            "domains": list(receipt.domains),
        },
    }


def _press_production(source) -> dict:
    CAPTURED.clear()
    _select_seams(source)
    before = dict(_builds())
    started = time.perf_counter()
    outcome = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    wall = time.perf_counter() - started
    bpy.ops.object.mode_set(mode="OBJECT")
    assert outcome == {"FINISHED"}, outcome
    after = _builds()
    settings = bpy.context.scene.hotspotuv_decal_mesh
    return {
        "wall_seconds": round(wall, 3),
        "status": settings.status,
        "timing_text": settings.timing,
        "builds_delta": {
            key: after.get(key, 0) - before.get(key, 0) for key in sorted(after)
        },
        "object": _object_stats(source.name),
        "run": _run_stats(),
    }


def _press_debug(source) -> dict:
    _select_seams(source)
    before = dict(_builds())
    started = time.perf_counter()
    outcome = bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
    wall = time.perf_counter() - started
    bpy.ops.object.mode_set(mode="OBJECT")
    assert outcome == {"FINISHED"}, outcome
    after = _builds()
    return {
        "wall_seconds": round(wall, 3),
        "builds_delta": {
            key: after.get(key, 0) - before.get(key, 0) for key in sorted(after)
        },
    }


def _write_obj(source_name: str, path: Path) -> dict:
    from cftuv.envelope_production_mesh import decal_object_name

    decal = bpy.data.objects[decal_object_name(source_name)]
    mesh = decal.data
    layer = mesh.uv_layers["UVMap"]
    lines = [
        f"# CFTUV decal mesh of {source_name} (source-local coordinates)",
        f"o {decal.name}",
    ]
    lines += [f"v {v.co.x:.5f} {v.co.y:.5f} {v.co.z:.5f}" for v in mesh.vertices]
    lines += [f"vt {item.uv[0]:.5f} {item.uv[1]:.5f}" for item in layer.data]
    for polygon in mesh.polygons:
        corners = [
            f"{mesh.loops[loop].vertex_index + 1}/{loop + 1}"
            for loop in polygon.loop_indices
        ]
        lines.append("f " + " ".join(corners))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"obj": str(path), "bytes": path.stat().st_size}


def _step(label: str, source, args) -> dict:
    if label == "cold":
        _reset_session()
        step = _press_production(source)
    elif label == "rebuild":
        step = _press_production(source)
    elif label == "debug_warm":
        _reset_session()
        debug = _press_debug(source)
        step = _press_production(source)
        step["debug_press"] = debug
    else:
        raise ValueError(label)
    step["label"] = label
    return step


def main() -> None:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root)
    _install_capture()
    source = bpy.data.objects[args.mesh]
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = args.alpha
    settings.envelope_debug_fan_density = args.density
    settings.envelope_debug_workers = args.workers
    result = {
        "root": str(root),
        "workers": args.workers,
        "mesh": args.mesh,
        "density": args.density,
        "alpha": args.alpha,
        "steps": [],
    }
    failure = None
    try:
        for label in args.steps.split(","):
            step = _step(label, source, args)
            result["steps"].append(step)
            if args.obj and label == "cold":
                result["obj"] = _write_obj(source.name, Path(args.obj))
            if args.export_json and label == "cold":
                from cftuv.envelope_production_export import export_production_json

                summary = export_production_json(
                    CAPTURED["run"].results,
                    Path(args.export_json),
                    label=args.mesh.replace(".", "_"),
                )
                result["export_json"] = str(summary)
            print(
                f"STEP {label}: wall {step['wall_seconds']} s | {step['status']} | "
                f"builds {step['builds_delta']} | {step['object'].get('faces')} faces"
            )
    except Exception:  # noqa: BLE001 - причина идёт в JSON
        failure = traceback.format_exc()
        print(failure)
    result["failure"] = failure
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(
        json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("PROBE_DONE" if failure is None else "PROBE_FAILED")


main()
