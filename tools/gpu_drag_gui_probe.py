"""gpu_drag_gui_probe: per-redraw gaps of the width drag in the OWNER'S live Blender (GPU_AFFINE_DRAG_V1 on vs off). Saves nothing, changes no setting, starts no process.

    python tools/blender_remote.py run --file tools/gpu_drag_gui_probe.py --script-args -- [out_dir]
    -> the owner drags once with "GPU affine drag" OFF and once with it ON (same object, same kind of motion, pause, LMB);
    python tools/blender_remote.py exec --code "import bpy; bpy.app.driver_namespace['cftuv_gpu_probe_stop']()"
    -> <out_dir>/cftuv_gpu_probe_<unix>.json (out_dir: the temp directory by default) and one summary line per drag in the console.

WHAT IT RECORDS (wrappers around module attributes, removed by the stop): the moment and the duration of every Python mesh write (`write_preview_geometry`: the model frame or the base,
`rebuild_mesh_from_image`), every node input set (`set_width`) and node install (`install`), every applied exact result (`envelope_width_live._apply`: a drag tick or the COMMIT),
every `depsgraph_update_post`, and every viewport redraw (`SpaceView3D` POST_PIXEL). A drag is the span between `begin_adjust` and `finish_adjust` of the modal operator; it remembers the
switches it ran with. Per drag: the redraw gaps (p50 / p90 / p99 / max, how many above 33 and 50 ms) and, for every gap above 50 ms, the Python work inside it by kind. The part of a gap
that no Python covers is Blender's own (the depsgraph: copy of the mesh, modifiers, normals; the GPU batch; the redraw itself).
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

import bpy

import cftuv.envelope_production_mesh as production_mesh
import cftuv.envelope_width_gpu_drag as gpu
import cftuv.envelope_width_live as live
import cftuv.envelope_width_modal as modal

KEY = "cftuv_gpu_probe"
_OUT = Path(sys.argv[sys.argv.index("--") + 1]) if "--" in sys.argv and len(sys.argv) > sys.argv.index("--") + 1 else Path(tempfile.gettempdir())

previous = bpy.app.driver_namespace.get(KEY + "_stop")
if previous is not None:  # a probe armed earlier is stopped first: one set of wrappers at a time
    try:
        previous()
    except Exception as exc:  # noqa: BLE001
        print("[CFTUV][gpu-probe] previous stop failed:", type(exc).__name__, exc, flush=True)

REC = {"python": [], "deps": [], "draw": [], "drags": []}
_ORIGINAL = []


def _wrap(module, name: str, kind: str, *, around=None):
    original = getattr(module, name)

    def wrapper(*args, **kwargs):
        started = time.perf_counter()
        try:
            return original(*args, **kwargs)
        finally:
            REC["python"].append((started, time.perf_counter() - started, kind if around is None else around(args)))

    setattr(module, name, wrapper)
    _ORIGINAL.append((module, name, original))


def _apply_kind(args) -> str:
    request = args[1] if len(args) > 1 else None
    grade = getattr(getattr(request, "payload", None), "grade", "COMMIT")
    return "apply_commit" if grade == "COMMIT" else "apply_tick"


def _switches() -> dict:
    settings = getattr(bpy.context.scene, "hotspotuv_decal_mesh", None)
    return {name: bool(getattr(settings, name, False)) for name in ("drag_gpu_affine", "drag_live_mesh", "drag_settle_on_pause")}


def _begin_wrapper(original):
    def begin(*args, **kwargs):
        REC["drags"].append({"begin": time.perf_counter(), "end": None, "switches": _switches()})
        return original(*args, **kwargs)

    return begin


def _finish_wrapper(original):
    def finish(*args, **kwargs):
        try:
            return original(*args, **kwargs)
        finally:
            if REC["drags"] and REC["drags"][-1]["end"] is None:
                REC["drags"][-1]["end"] = time.perf_counter()
                print("[CFTUV][gpu-probe]", json.dumps(_summary(REC["drags"][-1])), flush=True)

    return finish


for module, name, kind in (
    (production_mesh, "write_preview_geometry", "python_write"),
    (production_mesh, "rebuild_mesh_from_image", "python_rebuild"),
    (gpu, "set_width", "node_width"),
    (gpu, "install", "node_install"),
):
    _wrap(module, name, kind)
_wrap(live, "_apply", "apply", around=_apply_kind)
for name, factory in (("begin_adjust", _begin_wrapper), ("finish_adjust", _finish_wrapper)):
    original = getattr(modal, name)
    setattr(modal, name, factory(original))
    _ORIGINAL.append((modal, name, original))


def _deps(scene, depsgraph):
    REC["deps"].append(time.perf_counter())


def _draw():
    REC["draw"].append(time.perf_counter())


bpy.app.handlers.depsgraph_update_post.append(_deps)
_HANDLE = bpy.types.SpaceView3D.draw_handler_add(_draw, (), "WINDOW", "POST_PIXEL")


def _pct(values, q):
    values = sorted(values)
    return None if not values else round(values[min(len(values) - 1, int(q * len(values)))], 5)


def _summary(drag) -> dict:
    begin, end = drag["begin"], drag["end"] or time.perf_counter()
    draws = [moment for moment in REC["draw"] if begin <= moment <= end]
    gaps = [(a, b - a) for a, b in zip(draws, draws[1:])]
    python = [item for item in REC["python"] if begin <= item[0] <= end]
    big = []
    for at, gap in gaps:
        if gap <= 0.05:
            continue
        inside = [(kind, seconds) for started, seconds, kind in python if at <= started <= at + gap]
        kinds: dict = {}
        for kind, seconds in inside:
            kinds[kind] = round(kinds.get(kind, 0.0) + seconds, 5)
        big.append({"at": round(at - begin, 4), "gap": round(gap, 5), "python": kinds, "blender": round(gap - sum(kinds.values()), 5)})
    by_kind: dict = {}
    for _started, seconds, kind in python:
        by_kind.setdefault(kind, []).append(seconds)
    values = [gap for _at, gap in gaps]
    return {
        "switches": drag["switches"], "seconds": round(end - begin, 3), "draws": len(draws), "fps": round(len(draws) / max(end - begin, 1e-9), 1),
        "gap_p50": _pct(values, 0.5), "gap_p90": _pct(values, 0.9), "gap_p99": _pct(values, 0.99), "gap_max": round(max(values), 5) if values else None,
        "gaps_over_33ms": sum(1 for value in values if value > 0.033), "gaps_over_50ms": len(big),
        "big_gaps_blender_mostly": sum(1 for item in big if item["blender"] > 0.5 * item["gap"]),
        "python": {kind: {"n": len(items), "p50": _pct(items, 0.5), "p90": _pct(items, 0.9), "max": round(max(items), 5)} for kind, items in sorted(by_kind.items())},
        "depsgraph_updates": sum(1 for moment in REC["deps"] if begin <= moment <= end), "big_gaps": big,
    }


def stop():
    for module, name, original in reversed(_ORIGINAL):
        setattr(module, name, original)
    _ORIGINAL.clear()
    while _deps in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_deps)
    try:
        bpy.types.SpaceView3D.draw_handler_remove(_HANDLE, "WINDOW")
    except (ValueError, RuntimeError):
        pass
    bpy.app.driver_namespace.pop(KEY + "_stop", None)
    drags = [_summary(drag) for drag in REC["drags"]]
    path = _OUT / f"cftuv_gpu_probe_{int(time.time())}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"drags": drags, "draws": len(REC["draw"])}, indent=1), encoding="utf-8")
    for drag in drags:
        print("[CFTUV][gpu-probe]", json.dumps({key: value for key, value in drag.items() if key != "big_gaps"}), flush=True)
    print("[CFTUV][gpu-probe] ->", path, flush=True)
    return drags


bpy.app.driver_namespace[KEY + "_stop"] = stop
print("[CFTUV][gpu-probe] armed: drag once with 'GPU affine drag' off and once on, then call bpy.app.driver_namespace['cftuv_gpu_probe_stop']()", flush=True)
