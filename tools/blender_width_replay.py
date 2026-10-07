"""blender_width_replay: перетаскивание ширины на настоящих мешах сцены, headless, теми же функциями, что у модального инструмента.

    blender -b E:/testScene.blend --python-exit-code 1 --python tools/blender_width_replay.py -- \\
        --meshes rounded_wall_noise_top:0.25,sagging_wall:0.25,building:0.2239 --workers 8 --density 2 --stretch 42 --out replay.json

Для каждого меша: холодная кнопка «Build Decal Mesh», вход инструмента (`begin_adjust`: затравка сертификата в фоне), затем фазы:

1. `early_drag` — рука тянет СРАЗУ, не дожидаясь затравки: кадры идут, пока поток затравки считает (борьба за GIL), меш начинает двигаться,
   когда сертификат лёг (`first_mesh_frame_after_s`); цена событий и самая долгая остановка главного потока;
2. `frames` — воспроизведение перетаскивания (`WidthAdjustSessionV1.handle` -> `apply_step`) по пути вверх на `--up` и назад ниже базы,
   `--frames` кадров с паузой `--frame-ms`; между кадрами шагают таймеры, как их шагал бы главный цикл. `frame_ms` — цена кадра превью меша
   (p50, p95, максимум), `event_ms` — события целиком (линии + меш), `stall_ms_max` — самая долгая остановка главного потока, `live_domains` —
   сколько доменов двигалось и `held_domains_max` — сколько придержано;
3. `certificate` — домены, степень, байты сертификата, время до готовности затравки;
4. `deviation` — предсказание сертификата против ТОЧНОГО прогона на ряде ширин пути (`envelope_width_certificate.deviation`): наибольшее
   отклонение позиций меша (метры) и UV; `readback_max_abs_m` — меш, прочитанный из Blender (float32), против кадра;
5. `cancel` — отмена после перетаскивания возвращает меш побитово (`mesh_content_digest` и массивы float32);
6. `confirm` — подтверждение: точный результат побитово равен холодной кнопке на той же ширине, цена применения и счёта;
7. `slider` — ползунок: каждое значение идёт через калбэк `update` (линии, кадр, заказ точного счёта) под нагрузкой точных прогонов.

Ничего не сохраняется. Последняя строка при успехе: `WIDTH_REPLAY_OK`.
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

ROOT = Path(__file__).resolve().parents[1]


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--meshes", default="rounded_wall_noise_top:0.25,sagging_wall:0.25,building:0.2239")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--density", default="2")
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--up", type=float, default=0.30, help="путь вверх от базовой ширины, доля")
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument("--frame-ms", type=float, default=16.0)
    parser.add_argument("--checks", type=int, default=8)
    parser.add_argument("--out", default="")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


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
    for path in (root / "kernel" / "src", root):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve()
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


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def _stats(values):
    if not values:
        return {}
    return {
        "n": len(values),
        "p50": round(statistics.median(values), 3),
        "p95": round(_percentile(values, 0.95), 3),
        "max": round(max(values), 3),
    }


def _path(base, up, frames):
    """Ширина по кадрам: от базы вверх на `up` (доля), затем назад и вниз на треть `up` ниже базы."""

    half = frames // 2
    values = [base * (1.0 + up * (index + 1) / half) for index in range(half)]
    rest = frames - half
    top = values[-1]
    low = base * (1.0 - up / 3.0)
    values.extend(top + (low - top) * (index + 1) / rest for index in range(rest))
    return values


def _decal_mesh(obj_name):
    from cftuv.envelope_production_mesh import decal_object_name

    return bpy.data.objects[decal_object_name(obj_name)].data


def _digest(obj_name):
    from cftuv.envelope_production_mesh import mesh_content_digest

    return mesh_content_digest(_decal_mesh(obj_name))


def _mesh_arrays(obj_name):
    import numpy as np

    mesh = _decal_mesh(obj_name)
    co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get("co", co)
    layer = mesh.uv_layers["UVMap"]
    uv = np.empty(len(layer.data) * 2, dtype=np.float32)
    layer.data.foreach_get("uv", uv)
    return co, uv


def _controller():
    return bpy.context.window_manager._cftuv_envelope_debug_session


def _schedulers(controller):
    return [item for item in (controller.width_live, controller.width_prime) if item is not None]


def _step_timers(controller, spent):
    """Один шаг каждой зарегистрированной функции планировщиков (то, что сделал бы главный цикл); миллисекунды в `spent`."""

    for scheduler in _schedulers(controller):
        if bpy.app.timers.is_registered(scheduler._callback):
            started = time.perf_counter()
            scheduler._callback()
            spent.append((time.perf_counter() - started) * 1000.0)


def _settle(controller, *, timeout=600.0):
    """Таймеры шагают, пока оба планировщика не опустеют."""

    started = time.perf_counter()
    while any(item.busy for item in _schedulers(controller)):
        assert time.perf_counter() - started < timeout, "schedulers did not settle"
        _step_timers(controller, [])
        time.sleep(0.005)


def _press(obj):
    _select_seams(obj)
    started = time.perf_counter()
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
    return time.perf_counter() - started


def _pace(args, tick):
    time.sleep(max(0.0, args.frame_ms / 1000.0 - (time.perf_counter() - tick)))


def _certificate_record(controller, started):
    cert = controller.width_certificate
    return {
        "ready_s": round(time.perf_counter() - started, 3),
        "present": cert is not None,
        "refusal": None if controller.width_certificate_refusal is None else controller.width_certificate_refusal.outcome,
        "domains": None if cert is None else cert.domain_count,
        "certified": None if cert is None else cert.certified_domains,
        "quadratic": None if cert is None else cert.quadratic_domains,
        "own_bytes": None if cert is None else cert.own_bytes,
        "total_bytes": None if cert is None else cert.nbytes,
        "reasons": None if cert is None else cert.reason_counts(),
        "primes": controller.width_preview_log.primes_requested,
        "status": controller.width_prime.status_text if controller.width_prime else "",
    }


def _early_drag(runtime, base, args, result):
    """Рука тянет сразу после входа в инструмент: кадры идут, пока считает затравка; ждём, пока сертификат ляжет."""

    from cftuv.envelope_width_adjust import KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import apply_step

    controller = _controller()
    started = time.perf_counter()
    events, stalls, first_mesh, count = [], [], None, 0
    while any(item.busy for item in _schedulers(controller)):
        tick = time.perf_counter()
        apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_MOVE, base * 0.003 * (count % 8), 0.0)))
        elapsed = (time.perf_counter() - tick) * 1000.0
        events.append(elapsed)
        if controller.width_mesh_preview is not None and first_mesh is None:
            first_mesh = time.perf_counter() - started
        spent = []
        _step_timers(controller, spent)
        stalls.append(max([elapsed, *spent]))
        count += 1
        _pace(args, tick)
    apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_MOVE, 0.0, 0.0)))  # рука вернулась к исходной ширине
    _settle(controller)
    result["early_drag"] = {
        "frames_before_the_certificate_settled": count,
        "event_ms": _stats(events),
        "stall_ms_max": round(max(stalls), 3) if stalls else 0.0,
        "first_mesh_frame_after_s": None if first_mesh is None else round(first_mesh, 3),
    }
    result["certificate"] = _certificate_record(controller, started)
    return controller.width_certificate


def _drag(runtime, base, args, result):
    """Воспроизведение перетаскивания по пути ширины: цена кадров, событий и остановок главного потока."""

    from cftuv.envelope_width_adjust import KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_live import status_lines
    from cftuv.envelope_width_session import apply_step

    controller = _controller()
    events, frames, timers, stalls, live, held = [], [], [], [], [], []
    values = _path(base, args.up, args.frames)
    for width in values:
        tick = time.perf_counter()
        # Один пиксель — один метр, мышь стартует на оси: радиус события = смещение ширины от стартовой.
        apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_MOVE, width - base, 0.0)))
        elapsed = (time.perf_counter() - tick) * 1000.0
        events.append(elapsed)
        state = controller.width_mesh_preview
        if state is not None:
            frames.append(state.seconds * 1000.0)
            live.append(state.live)
            held.append(state.held)
        spent = []
        _step_timers(controller, spent)
        timers.extend(spent)
        stalls.append(max([elapsed, *spent]))
        _pace(args, tick)
    result["frames"] = {
        "replayed": len(values),
        "with_mesh": len(frames),
        "frame_ms": _stats(frames),
        "event_ms": _stats(events),
        "timer_ms_max": round(max(timers), 3) if timers else 0.0,
        "stall_ms_max": round(max(stalls), 3),
        "live_domains": [min(live), max(live)] if live else [],
        "held_domains_max": max(held) if held else 0,
        "last_state": None if controller.width_mesh_preview is None else controller.width_mesh_preview.outcome,
        "status": list(status_lines(controller)),
    }


def _deviation(cert, base, args):
    """Предсказание сертификата против точного прогона на ряде ширин пути; запись float32 против кадра."""

    import numpy as np

    from cftuv import envelope_width_certificate as certificate_module
    from cftuv.envelope_production_export import run_production
    from cftuv.envelope_production_mesh import build_mesh_arrays, write_preview_geometry
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    controller = _controller()
    record = controller.width_build
    mesh = _decal_mesh(record.source_name)
    offset = float(bpy.context.scene.hotspotuv_decal_mesh.offset)
    rows, readback = [], []
    for index in range(1, args.checks + 1):
        width = base * (1.0 + args.up * index / args.checks)
        frame = certificate_module.evaluate(cert, width)
        if frame.refusal == "":
            write_preview_geometry(mesh, frame.positions, frame.uvs)
            co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
            mesh.vertices.foreach_get("co", co)
            readback.append(float(np.max(np.abs(co - frame.positions))))
        exact = run_production(
            controller,
            record.analysis_bundle,
            record.selected,
            width,
            source_object_key=record.source_object_key,
            source_data_key=record.source_data_key,
            density=record.density,
            developable_stretch_budget=envelope_stretch_budget(record.stretch_percent),
            silhouette_uv_slide=envelope_dissolve_uv_slide(record.dissolve_percent),
            kernel_backend=record.kernel_backend,
            quiesce=False,
            workers=args.workers,
        )
        arrays = build_mesh_arrays(exact.results, offset)
        check = certificate_module.deviation(
            cert, certificate_module.sample_of(exact.results, arrays, key=cert.key, alpha_text=str(float(width)))
        )
        rows.append(
            {
                "width": round(width, 6),
                "relative": round((width - base) / base, 4),
                "max_position_m": check.max_position,
                "max_uv": check.max_uv,
                "checked": check.domains_checked,
                "skipped": check.domains_skipped,
                "refuted": len(check.refuted),
                "worst_patch": check.worst_patch,
                "live": frame.live_domains,
                "held": frame.held_domains,
            }
        )
    return rows, (max(readback) if readback else None)


def _cancel(runtime, base, args, before, before_arrays):
    """Отмена после перетаскивания: меш (до отмены сдвинутый) побитово прежний."""

    import numpy as np

    from cftuv.envelope_width_adjust import KIND_CANCEL, WidthEventV1
    from cftuv.envelope_width_mesh_preview import preview_mesh_now
    from cftuv.envelope_width_session import apply_step, finish_adjust

    controller = _controller()
    name = controller.width_build.source_name
    preview_mesh_now(controller, base * (1.0 + args.up))
    moved = not np.array_equal(_mesh_arrays(name)[0], before_arrays[0])
    apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_CANCEL, 0.0, 0.0)))
    outcome = finish_adjust(bpy.context, runtime, confirmed=False)
    after = _mesh_arrays(name)
    restored = _digest(name) == before and np.array_equal(after[0], before_arrays[0]) and np.array_equal(after[1], before_arrays[1])
    return {"moved_before_cancel": moved, "outcome": outcome, "restored_bitwise": bool(restored)}


def _confirm(obj, base, args):
    """Подтверждение: один точный пересчёт, меш равен холодной кнопке на той же ширине."""

    from cftuv.envelope_width_adjust import KIND_CONFIRM, KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import ViewScaleV1, apply_step, begin_adjust, finish_adjust

    controller = _controller()
    settings = bpy.context.scene.hotspotuv_settings
    runtime = begin_adjust(bpy.context, (0.0, 0.0), ViewScaleV1((0.0, 0.0), 1.0))
    final = base * (1.0 + args.up * 0.5)
    for width in (base * 1.01, base * 1.05, final):
        apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_MOVE, width - base, 0.0)))
    apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_CONFIRM, final - base, 0.0)))
    started = time.perf_counter()
    finish_adjust(bpy.context, runtime, confirmed=True)
    spent = []
    while controller.width_live is not None and controller.width_live.busy:
        _step_timers(controller, spent)
        time.sleep(0.005)
    seconds = time.perf_counter() - started
    live_digest = _digest(obj.name)
    applied = controller.width_live.last_applied
    _settle(controller)
    width = float(settings.envelope_debug_alpha)
    controller.clear()
    controller.width_live = None
    controller.width_prime = None
    settings.envelope_debug_alpha = width
    _press(obj)
    return {
        "width": width,
        "exact_ready_s": round(seconds, 3),
        "exact_apply_ms": None if applied is None else round(applied.apply_seconds * 1000.0, 1),
        "exact_compute_ms": None if applied is None else round(applied.compute_seconds * 1000.0, 1),
        "exact_timer_ms_max": round(max(spent), 1) if spent else 0.0,
        "equal_to_cold_button": _digest(obj.name) == live_digest,
    }


def _slider(base, args):
    """Ползунок: каждое значение идёт через калбэк `update` (линии, кадр, заказ точного счёта) под нагрузкой точных прогонов."""

    from cftuv.envelope_width_live import ensure_prime

    controller = _controller()
    settings = bpy.context.scene.hotspotuv_settings
    ensure_prime(bpy.context)
    _settle(controller)
    values = [base * (1.0 + 0.004 * index) for index in range(1, 21)] + [base * (1.0 + 0.004 * index) for index in range(20, -1, -1)]
    changes, timers, frames = [], [], []
    for value in values:
        tick = time.perf_counter()
        settings.envelope_debug_alpha = value
        changes.append((time.perf_counter() - tick) * 1000.0)
        if controller.width_mesh_preview is not None:
            frames.append(controller.width_mesh_preview.seconds * 1000.0)
        _step_timers(controller, timers)
        _pace(args, tick)
    _settle(controller)
    return {
        "updates": len(values),
        "callback_ms": _stats(changes),
        "frame_ms": _stats(frames),
        "timer_ms_max": round(max(timers), 3) if timers else 0.0,
        "applied": controller.width_live.counters.applied,
        "failed": controller.width_live.counters.failed,
    }


def _replay(obj, base, args, result):
    from cftuv.envelope_width_session import ViewScaleV1, begin_adjust

    before, before_arrays = _digest(obj.name), _mesh_arrays(obj.name)  # меш кнопки: отмена обязана вернуть его побитово
    runtime = begin_adjust(bpy.context, (0.0, 0.0), ViewScaleV1((0.0, 0.0), 1.0))
    assert not isinstance(runtime, str), runtime
    cert = _early_drag(runtime, base, args, result)
    if cert is None:
        result["failure"] = "no certificate: " + str(result["certificate"]["refusal"])
        return
    _drag(runtime, base, args, result)
    result["deviation"], result["readback_max_abs_m"] = _deviation(cert, base, args)
    result["cancel"] = _cancel(runtime, base, args, before, before_arrays)
    result["confirm"] = _confirm(obj, base, args)
    result["slider"] = _slider(result["confirm"]["width"], args)


def _summary(name, result) -> str:
    frames, certificate = result["frames"], result["certificate"]
    worst = max((item["max_position_m"] for item in result["deviation"]), default=0.0)
    early, slider = result["early_drag"], result["slider"]
    return (
        f"REPLAY {name}: frame {frames['frame_ms']} ms, event {frames['event_ms']} ms, stall max {frames['stall_ms_max']} ms, "
        f"certificate {certificate['certified']}/{certificate['domains']} domains {certificate['own_bytes']} B in {certificate['ready_s']} s, "
        f"max deviation {worst:.3e} m, cancel {result['cancel']['restored_bitwise']}, confirm == cold {result['confirm']['equal_to_cold_button']}\n"
        f"REPLAY {name}: immediate drag while the prime computes: event {early['event_ms']} ms, stall max {early['stall_ms_max']} ms, "
        f"first mesh frame after {early['first_mesh_frame_after_s']} s; slider: callback {slider['callback_ms']} ms, "
        f"frame {slider['frame_ms']} ms; exact apply {result['confirm']['exact_apply_ms']} ms"
    )


def main() -> None:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root)
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_workers = args.workers
    settings.envelope_debug_fan_density = args.density
    settings.envelope_debug_max_stretch = args.stretch
    report = {"root": str(root), "workers": args.workers, "density": args.density, "stretch": args.stretch, "meshes": {}}
    failures = []
    for entry in args.meshes.split(","):
        name, _, text = entry.partition(":")
        base = float(text or 0.25)
        controller = getattr(bpy.context.window_manager, "_cftuv_envelope_debug_session", None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            bpy.context.window_manager._cftuv_envelope_debug_session = controller
        controller.clear()
        controller.width_live = None
        controller.width_prime = None
        settings.envelope_debug_alpha = base
        result = {"alpha": base}
        report["meshes"][name] = result
        try:
            obj = bpy.data.objects[name]
            result["cold_button_s"] = round(_press(obj), 3)
            decal = _decal_mesh(name)
            result["mesh"] = {"vertices": len(decal.vertices), "faces": len(decal.polygons), "loops": len(decal.loops)}
            _replay(obj, base, args, result)
        except Exception:  # noqa: BLE001 - причина идёт в JSON
            result["failure"] = traceback.format_exc()
        if result.get("failure"):
            failures.append(name)
            print(f"REPLAY_FAILED {name}\n{result['failure']}")
        else:
            print(_summary(name, result))
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("WIDTH_REPLAY_OK" if not failures else f"WIDTH_REPLAY_FAILED {failures}")


if __name__ == "__main__":
    main()
