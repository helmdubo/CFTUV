"""Замер живой ширины декали на настоящем меше сцены (DECAL-WIDTH-LIVE): превью, точный пересчёт, равенство.

    blender -b E:\\testscene.blend --python-exit-code 1 --python measure_probe.py -- \\
        --root <дерево> --out <json> [--mesh building] [--workers 8] [--density 2] [--drags 3] [--cold]

Аддон берётся ИЗ ДЕРЕВА `--root` (установленный снимается). Таймеры `bpy.app.timers` в фоновом Blender не
срабатывают, поэтому зонд ШАГАЕТ таймер сам (как зонд ASYNC-ALPHA): вызывает функцию, которую планировщик
зарегистрировал (`scheduler._callback`), и спит, сколько она просит.

В JSON, на каждое перетаскивание из пяти значений через путь `update` ползунка после «Build Decal Mesh»:

- `change_ms` — секунды главного потока на КАЖДОЕ изменение свойства (заказ вместе с мгновенным превью);
- `preview_ms` — цена одной чистой функции превью (`compute_width_preview`) на последнем изменении;
- `latency_ms` — от ПОСЛЕДНЕГО изменения до применённого точного результата (пауза, счёт, применение);
- `compute_ms` / `apply_ms` — цена точного пересчёта в потоке и записи меша на главном потоке;
- `steps_ms` — секунды главного потока на каждый шаг таймера (`apply` — самый тяжёлый);
- `counters` — куда ушла каждая работа.

Отдельно: `preview_scan` — функция превью на ряде ширин (медиана и максимум по 30 вызовов), `capture` — цена
сбора входов превью в кнопке (один раз на прогон), `equal` — меш живого результата на последней ширине против
прямого нажатия кнопки (тёплая сессия; с `--cold` ещё и холодная), `cancel_return_ms` — за сколько
возвращается отменённый полёт.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy


DRAGS = (
    (0.30, 0.31, 0.32, 0.33, 0.34),
    (0.36, 0.37, 0.38, 0.39, 0.40),
    (0.42, 0.43, 0.44, 0.45, 0.46),
    (0.28, 0.27, 0.26, 0.25, 0.24),
)
SCAN_WIDTHS = (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0)
HAND_PAUSE_SECONDS = 0.016


def _arguments() -> argparse.Namespace:
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--mesh", default="building")
    parser.add_argument("--density", default="2")
    parser.add_argument("--drags", type=int, default=3)
    parser.add_argument("--cold", action="store_true")
    return parser.parse_args(tail)


def _load_tree(root: Path):
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
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


def _digest(obj_name: str) -> str:
    from cftuv.envelope_production_mesh import decal_object_name, mesh_content_digest

    return mesh_content_digest(bpy.data.objects[decal_object_name(obj_name)].data)


def _pump(scheduler, *, timeout=600.0):
    callback = scheduler._callback
    steps = []
    oversleep = 0.0
    end = time.perf_counter() + timeout
    while True:
        started = time.perf_counter()
        delay = callback()
        steps.append(time.perf_counter() - started)
        if delay is None:
            break
        assert time.perf_counter() < end, "width live did not settle"
        asked = time.perf_counter()
        time.sleep(delay)
        oversleep = max(oversleep, time.perf_counter() - asked - delay)
    if not scheduler.busy and bpy.app.timers.is_registered(callback):
        bpy.app.timers.unregister(callback)
    return steps, oversleep


def _drag(controller, settings, values):
    changes = []
    for value in values:
        started = time.perf_counter()
        settings.envelope_debug_alpha = value
        changes.append(time.perf_counter() - started)
        time.sleep(HAND_PAUSE_SECONDS)
    state = controller.width_preview
    scheduler = controller.width_live
    before = scheduler.counters
    steps, oversleep = _pump(scheduler)
    after = scheduler.counters
    applied = scheduler.last_applied
    return {
        "values": list(values),
        "change_ms": [round(item * 1000, 3) for item in changes],
        "preview_ms": None if state is None else round(state.preview.seconds * 1000, 3),
        "preview_lines": None if state is None else state.preview.lines,
        "preview_points": None if state is None else state.preview.points,
        "preview_outcomes": None if state is None else dict(state.preview.outcomes),
        "latency_ms": round(applied.latency_seconds * 1000, 1),
        "compute_ms": round(applied.compute_seconds * 1000, 1),
        "apply_ms": round(applied.apply_seconds * 1000, 1),
        "steps_ms": [round(item * 1000, 2) for item in steps],
        "oversleep_ms": round(oversleep * 1000, 2),
        "counters": {name: getattr(after, name) - getattr(before, name) for name in after.__slots__},
        "status": scheduler.status_text,
        "decal_timing": str(bpy.context.scene.hotspotuv_decal_mesh.timing),
    }


def _cancel_scenario(controller, settings):
    from cftuv.envelope_width_live import scheduler_of

    scheduler = scheduler_of(controller)
    before = scheduler.counters
    settings.envelope_debug_alpha = 0.41
    start = time.perf_counter()
    while not scheduler.in_flight:
        assert time.perf_counter() - start < 60.0
        scheduler._callback()
        time.sleep(0.01)
    time.sleep(0.05)
    job = scheduler._job
    requested = time.perf_counter()
    settings.envelope_debug_alpha = 0.47
    while not job.finished:
        time.sleep(0.001)
    cancel_return = time.perf_counter() - requested
    steps, _oversleep = _pump(scheduler)
    after = scheduler.counters
    return {
        "cancel_return_ms": round(cancel_return * 1000, 1),
        "counters": {name: getattr(after, name) - getattr(before, name) for name in after.__slots__},
        "steps_max_ms": round(max(steps) * 1000, 2),
        "status": scheduler.status_text,
    }


def _preview_scan(controller):
    from cftuv.envelope_width_preview import compute_width_preview

    inputs = controller.width_build.preview_inputs
    rows = {}
    for width in SCAN_WIDTHS:
        seconds = []
        for _ in range(30):
            preview = compute_width_preview(inputs, width, lift=0.02)
            seconds.append(preview.seconds * 1000.0)
        rows[str(width)] = {
            "median_ms": round(statistics.median(seconds), 3),
            "max_ms": round(max(seconds), 3),
            "lines": preview.lines,
            "points": preview.points,
            "outcomes": dict(preview.outcomes),
        }
    return rows


def main() -> None:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root)
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    obj = bpy.data.objects[args.mesh]
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
    result = {"root": str(root), "mesh": args.mesh, "workers": args.workers, "density": args.density}
    failure = None
    try:
        selected = _select_seams(obj)
        result["selected_edges"] = selected
        started = time.perf_counter()
        assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
        result["cold_button_s"] = round(time.perf_counter() - started, 3)
        started = time.perf_counter()
        assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
        result["warm_button_s"] = round(time.perf_counter() - started, 3)
        record = controller.width_build
        inputs = record.preview_inputs
        result["capture"] = {
            "ms": round(inputs.build_seconds * 1000, 1),
            "runs": len(inputs.runs),
            "sides": sum(len(item.sides) for item in inputs.runs),
            "edges": inputs.edges,
            "faces": len(inputs.faces),
            "named": dict(inputs.outcomes),
        }
        result["decal_timing"] = str(bpy.context.scene.hotspotuv_decal_mesh.timing)
        result["preview_scan"] = _preview_scan(controller)

        result["drags"] = [_drag(controller, settings, values) for values in DRAGS[: args.drags]]
        last = float(settings.envelope_debug_alpha)
        live = _digest(args.mesh)
        started = time.perf_counter()
        assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
        result["button_at_last_width_s"] = round(time.perf_counter() - started, 3)
        equal = {"warm": _digest(args.mesh) == live}
        if args.cold:
            controller.clear()
            controller.width_live = None
            _select_seams(obj)
            started = time.perf_counter()
            assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
            result["cold_button_at_last_width_s"] = round(time.perf_counter() - started, 3)
            equal["cold"] = _digest(args.mesh) == live
        result["last_width"] = last
        result["equal"] = equal
        result["cancel"] = _cancel_scenario(controller, settings)
        latencies = [item["latency_ms"] for item in result["drags"]]
        changes = [value for item in result["drags"] for value in item["change_ms"]]
        steps = [value for item in result["drags"] for value in item["steps_ms"]]
        result["summary"] = {
            "latency_ms_median": statistics.median(latencies),
            "latency_ms_max": max(latencies),
            "change_ms_max": max(changes),
            "change_ms_median": statistics.median(changes),
            "preview_ms_median": statistics.median(item["preview_ms"] for item in result["drags"]),
            "step_ms_max": max(steps),
            "apply_ms_median": statistics.median(item["apply_ms"] for item in result["drags"]),
            "compute_ms_median": statistics.median(item["compute_ms"] for item in result["drags"]),
        }
        print("SUMMARY", json.dumps(result["summary"]), "equal", equal)
    except Exception:  # noqa: BLE001 - причина идёт в JSON
        failure = traceback.format_exc()
        print(failure)
    result["failure"] = failure
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("PROBE_DONE" if failure is None else "PROBE_FAILED")


main()
