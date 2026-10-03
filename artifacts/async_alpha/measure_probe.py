"""Замер фонового превью alpha на настоящем меше сцены (ASYNC-ALPHA): кнопка, перетаскивание, равенство.

    blender -b E:\\testscene.blend --python-exit-code 1 --python measure_probe.py -- \\
        --root <дерево> --out <json> [--mesh building] [--workers 8] [--density 2] [--drags 4]

Аддон берётся ИЗ ДЕРЕВА `--root` (установленный снимается). Таймеры `bpy.app.timers` в фоновом
Blender не срабатывают, поэтому зонд ШАГАЕТ таймер сам: вызывает функцию, которую планировщик
зарегистрировал (`scheduler._callback`), и спит, сколько она просит; недосып сверх заказанного —
оценка того, насколько поздно главный поток проснулся бы в интерфейсе (захват GIL потоком счёта).

Что пишется в JSON, на каждое перетаскивание из пяти значений через путь `update` ползунка:

- `change_ms` — секунды главного потока на КАЖДОЕ изменение свойства (заказ);
- `steps_ms` — секунды главного потока на каждый шаг таймера (`apply` — самый тяжёлый из них);
- `latency_ms` — от ПОСЛЕДНЕГО изменения до применённого результата (пауза, счёт, применение);
- `compute_ms` / `apply_ms` — цена счёта в потоке и применения на главном потоке;
- `oversleep_ms` — наибольший недосып главного потока между шагами;
- `counters` — куда ушла каждая работа.

После перетаскиваний кнопка отладки нажимается на ТОМ ЖЕ alpha: слои очереди, штрихи GP и запись
sidecar очереди обязаны совпасть побитово (`equal`). Отдельный сценарий отменяет полёт новым
значением и меряет, за сколько отменённый полёт возвращается (`cancel_return_ms`).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy


SECONDS_KEYS = ("prepare_seconds", "coverage_seconds", "contour_seconds", "timings")
DRAGS = (
    (0.30, 0.31, 0.32, 0.33, 0.34),
    (0.36, 0.37, 0.38, 0.39, 0.40),
    (0.42, 0.43, 0.44, 0.45, 0.46),
    (0.28, 0.27, 0.26, 0.25, 0.24),
)
HAND_PAUSE_SECONDS = 0.016


def _arguments() -> argparse.Namespace:
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--mesh", default="building")
    parser.add_argument("--density", default="2")
    parser.add_argument("--drags", type=int, default=4)
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


def _strip(value):
    if isinstance(value, dict):
        return {k: _strip(v) for k, v in value.items() if k not in SECONDS_KEYS}
    if isinstance(value, list):
        return [_strip(item) for item in value]
    return value


def _sha(value) -> str:
    text = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


def _gp_digest(gp_obj):
    rows = []
    for layer in gp_obj.data.layers:
        name = getattr(layer, "name", getattr(layer, "info", ""))
        if not name.startswith("ENV_9"):
            continue
        for frame in layer.frames:
            drawing = getattr(frame, "drawing", None)
            strokes = drawing.strokes if drawing is not None else frame.strokes
            for stroke in strokes:
                rows.append(
                    (
                        name,
                        int(getattr(stroke, "material_index", 0)),
                        bool(getattr(stroke, "cyclic", getattr(stroke, "use_cyclic", False))),
                        [
                            (tuple(getattr(point, "position", None) or point.co), round(float(point.radius), 9))
                            for point in stroke.points
                        ],
                    )
                )
    return {"strokes": len(rows), "digest": _sha(rows)}


def _answer(obj):
    from cftuv.envelope_debug_renderer import (
        envelope_debug_object_name,
        envelope_debug_text_name,
    )

    payload = json.loads(bpy.data.texts[envelope_debug_text_name(obj)].as_string())
    gp_obj = bpy.data.objects[envelope_debug_object_name(obj.name)]
    queue = _strip(payload["queue"])
    strokes = [item for item in payload["strokes"] if item.get("stage") == "QUEUE"]
    return {
        "queue": _sha(queue),
        "queue_strokes": _sha(strokes),
        "gp": _gp_digest(gp_obj),
        "alpha": queue["domains"][0]["alpha"] if queue.get("domains") else None,
    }


def _pump(scheduler, *, stop_when=None, timeout=300.0):
    callback = scheduler._callback
    steps = []
    oversleep = 0.0
    end = time.perf_counter() + timeout
    while True:
        if stop_when is not None and stop_when():
            break
        started = time.perf_counter()
        delay = callback()
        steps.append(time.perf_counter() - started)
        if delay is None:
            break
        assert time.perf_counter() < end, "alpha preview did not settle"
        asked = time.perf_counter()
        time.sleep(delay)
        oversleep = max(oversleep, time.perf_counter() - asked - delay)
    if not scheduler.busy and bpy.app.timers.is_registered(callback):
        bpy.app.timers.unregister(callback)
    return steps, oversleep


def _drag(settings, scheduler_getter, values):
    changes = []
    for value in values:
        started = time.perf_counter()
        settings.envelope_debug_alpha = value
        changes.append(time.perf_counter() - started)
        time.sleep(HAND_PAUSE_SECONDS)
    last_change = time.perf_counter() - HAND_PAUSE_SECONDS
    scheduler = scheduler_getter()
    before = scheduler.counters
    steps, oversleep = _pump(scheduler)
    done = time.perf_counter()
    after = scheduler.counters
    applied = scheduler.last_applied
    return {
        "values": list(values),
        "change_ms": [round(item * 1000, 3) for item in changes],
        "steps_ms": [round(item * 1000, 2) for item in steps],
        "latency_ms": round(applied.latency_seconds * 1000, 1),
        "wall_from_last_change_ms": round((done - last_change) * 1000, 1),
        "compute_ms": round(applied.compute_seconds * 1000, 1),
        "apply_ms": round(applied.apply_seconds * 1000, 1),
        "oversleep_ms": round(oversleep * 1000, 2),
        "counters": {
            name: getattr(after, name) - getattr(before, name)
            for name in after.__slots__
        },
        "status": scheduler.status_text,
        "queue_timing": str(bpy.context.scene.hotspotuv_settings.envelope_debug_queue_timing),
    }


def _cancel_scenario(settings, scheduler_getter):
    """Значение во время полёта: сколько отменённый полёт возвращается, и что применено в итоге."""

    scheduler = scheduler_getter()
    before = scheduler.counters
    settings.envelope_debug_alpha = 0.41
    start = time.perf_counter()
    while not scheduler.in_flight:
        assert time.perf_counter() - start < 60.0
        scheduler._callback()
        time.sleep(0.01)
    time.sleep(0.05)  # полёт идёт
    job = scheduler._job
    requested = time.perf_counter()
    settings.envelope_debug_alpha = 0.47
    while not job.finished:
        time.sleep(0.001)
    cancel_return = time.perf_counter() - requested
    steps, oversleep = _pump(scheduler)
    after = scheduler.counters
    return {
        "cancel_return_ms": round(cancel_return * 1000, 1),
        "counters": {name: getattr(after, name) - getattr(before, name) for name in after.__slots__},
        "steps_max_ms": round(max(steps) * 1000, 2),
        "status": scheduler.status_text,
    }


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

    def scheduler_getter():
        return controller.alpha_preview

    result = {"root": str(root), "mesh": args.mesh, "workers": args.workers, "density": args.density}
    failure = None
    try:
        _select_seams(obj)
        started = time.perf_counter()
        assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
        result["cold_button_s"] = round(time.perf_counter() - started, 3)
        started = time.perf_counter()
        assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
        result["warm_button_s"] = round(time.perf_counter() - started, 3)
        result["domains"] = len(controller.queue_session.entries)

        result["drags"] = [
            _drag(settings, scheduler_getter, values) for values in DRAGS[: args.drags]
        ]
        last_alpha = float(settings.envelope_debug_alpha)
        slider = _answer(obj)
        started = time.perf_counter()
        assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
        result["button_at_last_alpha_s"] = round(time.perf_counter() - started, 3)
        button = _answer(obj)
        result["last_alpha"] = last_alpha
        result["equal"] = {key: slider[key] == button[key] for key in ("queue", "queue_strokes", "gp")}
        result["slider_answer"] = slider
        result["button_answer"] = button

        result["cancel"] = _cancel_scenario(settings, scheduler_getter)
        latencies = [item["latency_ms"] for item in result["drags"]]
        changes = [value for item in result["drags"] for value in item["change_ms"]]
        steps = [value for item in result["drags"] for value in item["steps_ms"]]
        result["summary"] = {
            "latency_ms_median": statistics.median(latencies),
            "latency_ms_max": max(latencies),
            "change_ms_max": max(changes),
            "change_ms_median": statistics.median(changes),
            "step_ms_max": max(steps),
            "step_ms_median": statistics.median(steps),
            "apply_ms_median": statistics.median(item["apply_ms"] for item in result["drags"]),
            "compute_ms_median": statistics.median(item["compute_ms"] for item in result["drags"]),
            "oversleep_ms_max": max(item["oversleep_ms"] for item in result["drags"]),
        }
        print("SUMMARY", json.dumps(result["summary"]), "equal", result["equal"])
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
