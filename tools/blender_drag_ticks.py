"""blender_drag_ticks: the exact refreshes of a width drag, timed call for call on the live path (headless).

    blender -b scene.blend --python-exit-code 1 --python tools/blender_drag_ticks.py -- --root <tree> --native-path <dir> --case building \
        --workers 12 --grade commit|lean|incremental [--shadow] --ticks 20 --confirms 3 --out ticks.json

WHAT IT MEASURES. After the cold button (the operator), `--ticks` exact refreshes at growing widths (+0.5 % of the cold width each) and `--confirms` commits after them. A refresh is one
width step of the live path exactly as the product takes it: `envelope_width_live._begin` (main thread) -> `compute` in the thread (`run_production` + `finish_live_run`) -> `_validity` ->
`_apply` (main thread). The scheduler's debounce (0.15 s) and poll are left out: they are constants of the product. `--grade commit` is the full path (what the base tree always does);
`--grade lean` is WIDTH_REFRESH_LEAN_V1 (a tree that has `WidthLiveTargetV1.grade`) and needs the tool's drag to be active, which the script sets. `--grade incremental` is WIDTH_REFRESH_INCREMENTAL_V1 (a tree that has
`envelope_width_incremental`): the tick the tool orders while dragging; the policy of the tree decides per tick whether it is incremental or a full lean tick (the verification), and the record names which (`plan`).
`--shadow` (incremental only) additionally runs, after every tick and OUTSIDE its time, a full lean refresh at the same width that is never applied, and records how the incremental tick differs from it (served domains whose
structure digest differs from the exact one = flips the tick did not see, the largest position / UV deviation of the served and of the recomputed domains, whether the mesh layout is the same): the evidence for the verification
period. The confirms are always the full path:
their time is the time "from release to the exact mesh" (plus the debounce). `--mode object` (default) leaves the edit mode after the cold button, as a drag in the object mode does (the record of the
source check serves only that mode: the data of the edit mode is not in the mesh arrays yet); `--mode edit` drags in the edit mode.

WHAT IT PROVES (`--prove-lean K`, lean grade only). For the first K widths: a lean refresh, the digest of the mesh it wrote (`mesh_content_digest`), then the full path at the SAME width,
and its digest: they must be equal (a lean frame is the committed mesh, not an approximation), and the mesh after the cancel of the drag must be the mesh before it, bit for bit.

REPORT. One JSON: per refresh the wall time, the main-thread begin and apply, the thread time, the laps of the parent stages that repeat per refresh (scan, order_by_cost, weld, the
arrays, the digests, the model), p50 / p90 over the refreshes, the mesh digest and a hash of the final mesh arrays (two trees that give the same committed answer give the same hash).
Last line: DRAG_TICKS_OK. The scene is never saved and no Blender but this one is touched.

WHERE THE CYCLIC COLLECTOR WENT (GC_PAUSED_DURING_WIDTH_TICK_V1). `gc.callbacks` times every collection of the process: per refresh `gc_seconds` / `gc_collections` / `gc_gen2` inside its window (begin ->
end of apply), `gc` for the whole drag (`in_windows` and `outside`: a collection that a pause only DEFERRED lands in the gap after the apply, so it is counted there and in `tick_cycle`, the time from
the end of one refresh to the end of the next, not hidden), and `gc_enabled_after_step` (the idle between refreshes runs with the collector on: every value True). `rss_mb`: the working set of this (parent)
process: `ticks_peak` is the highest value a 5 ms sampler saw while the refreshes and the confirms ran, `process_peak` the OS peak of the whole run (the cold button included); `rss_after_mb` per
refresh is the same working set at the end of each (a leak of a long drag shows as a slope). `gc_slow` per refresh lists every collection above 20 ms as [generation, seconds, thread, offset from the
start of the refresh]: who pays for a collection and when (a tree without the pause: a generation-2 collection of 0.35-0.45 s in one of several threads; with it: one young collection of ~0.1 s in the
compute thread at the moment the pause ends).
"""

from __future__ import annotations

import argparse
import dataclasses
import gc
import hashlib
import json
import os
import statistics
import sys
import threading
import time
import traceback
from collections import Counter
from pathlib import Path

# SNAPSHOT_BYTE_CHECK_SAMPLED_V1: the gates and the measurements of the tools check every domain whole (the product button samples). It costs the cold press only: the width steps reuse the snapshots.
os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import blender_tree_ab as tree_ab  # noqa: E402  (the tree loader, the native check, the case recipes and the edge selection are shared with the A/B tool)

#: (module attribute path, lap name): the parent stages that repeat at every width step; a missing one (an older tree) is skipped.
LAPS = (
    ("cftuv.envelope_width_live", "_current_digest", "begin.source_digest"),
    ("cftuv.envelope_production_export", "_scan", "scan"),
    ("cftuv.envelope_production_export", "retry_snap_lottery", "retry"),
    ("cftuv.envelope_domain_pool", "order_by_cost", "pool.order_by_cost"),
    ("cftuv.envelope_production_mesh", "build_mesh_arrays", "arrays"),
    ("cftuv.envelope_production_mesh", "weld_vertices", "arrays.weld"),
    ("cftuv.envelope_production_mesh", "cross_domain_seams", "arrays.cross_seams"),
    ("cftuv.envelope_production_mesh", "_arrays_digest", "arrays.digest"),
    ("cftuv.envelope_production_mesh", "off_plane_after_offset", "arrays.off_plane"),
    ("cftuv.envelope_production_mesh", "seam_report_of_chains", "arrays.seam_report"),
    ("cftuv.envelope_production_mesh", "half_edge_conflicts", "arrays.half_edge"),
    ("cftuv.envelope_production_mesh", "rewrite_decal_mesh", "apply.rewrite"),
    ("cftuv.envelope_production_mesh", "mesh_content_digest", "apply.mesh_digest"),
    ("cftuv.envelope_width_live", "finish_live_run", "tail"),
    ("cftuv.envelope_width_live", "lean_exact_now", "apply.lean_frame"),
    ("cftuv.envelope_width_live", "plan_tick", "plan"),
    ("cftuv.envelope_width_mesh_preview", "rebase_result", "plan.rebase"),
    ("cftuv.envelope_width_mesh_preview", "build_sample", "tail.sample"),
    ("cftuv.envelope_width_mesh_preview", "_composite_origins", "tail.origins"),
    ("cftuv.envelope_width_mesh_preview", "compose_sample", "tail.compose"),
    ("cftuv.envelope_width_composite", "_weld_into", "tail.compose.weld"),
    ("cftuv.envelope_width_composite", "_reference_index", "tail.compose.refindex"),
    ("cftuv.envelope_width_composite", "_rows_after", "tail.compose.rows"),
    ("cftuv.envelope_production_export", "_dispatch", "dispatch"),
    ("cftuv.envelope_width_mesh_preview", "build_model", "tail.model"),
    ("cftuv.envelope_width_mesh_preview", "deviation", "tail.deviation"),
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--case", default="building")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--grade", default="commit", choices=("commit", "lean", "incremental"))
    parser.add_argument("--shadow", action="store_true", help="incremental only: after every tick, outside its time, a full lean refresh at the same width (never applied) and the difference between the two")
    parser.add_argument("--ticks", type=int, default=20)
    parser.add_argument("--confirms", type=int, default=3)
    parser.add_argument("--rel", type=float, default=0.005, help="each width step, as a share of the cold width")
    parser.add_argument("--prove-digest", action="store_true", help="the receipt digest of the decal mesh read by foreach_get equals the digest read item by item (the previous implementation, verbatim), and their times")
    parser.add_argument("--prove-lean", type=int, default=0)
    parser.add_argument("--prove-rel", type=float, default=0.001, help="the proof widths step, as a share of the cold width (small: a lean frame is writable while the layout is the one on screen)")
    parser.add_argument("--mode", default="object", choices=("object", "edit"), help="the mode of the source while the width is dragged: the button needs the edit mode (it reads the selection), the drag is usually made in the object mode")
    parser.add_argument("--kernel-backend", default="NATIVE")
    parser.add_argument("--out", required=True)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


class Laps:
    """Seconds and calls per lap name since the last `reset`; the wrappers add to it."""

    def __init__(self) -> None:
        self.seconds: dict = {}
        self.calls: dict = {}

    def reset(self) -> None:
        self.seconds, self.calls = {}, {}

    def snapshot(self) -> dict:
        return {name: [self.calls[name], round(self.seconds[name], 6)] for name in sorted(self.seconds)}


class GcTrace:
    """Seconds and counts of the cyclic collector (`gc.callbacks`): cumulative, and since the last `window_reset` (one refresh)."""

    def __init__(self) -> None:
        self._started = 0.0
        self.seconds, self.collections, self.gen2 = 0.0, 0, 0
        self._mark = (0.0, 0, 0)
        self.slow: list = []  # (start, seconds, generation, thread) of every collection above 20 ms: who pays, and when
        self._slow_mark = 0

    def __call__(self, phase: str, info: dict) -> None:
        if phase == "start":
            self._started = time.perf_counter()
            return
        spent = time.perf_counter() - self._started
        self.seconds += spent
        self.collections += 1
        self.gen2 += 1 if info.get("generation") == 2 else 0
        if spent > 0.02:
            self.slow.append((self._started, spent, info.get("generation"), threading.current_thread().name))

    def install(self) -> None:
        gc.callbacks.append(self)

    def window_reset(self) -> None:
        self._mark = (self.seconds, self.collections, self.gen2)
        self._slow_mark = len(self.slow)

    def window(self, origin: float = 0.0) -> dict:
        """The collections since `window_reset`; `gc_slow` lists those above 20 ms as `[generation, seconds, thread, start offset from `origin`]`."""

        slow = [[generation, round(spent, 4), thread, round(start - origin, 4)] for start, spent, generation, thread in self.slow[self._slow_mark :]]
        return {"gc_seconds": self.seconds - self._mark[0], "gc_collections": self.collections - self._mark[1], "gc_gen2": self.gen2 - self._mark[2], "gc_slow": slow}

    def cumulative(self) -> tuple:
        return (self.seconds, self.collections, self.gen2)


def working_set_bytes() -> tuple:
    """`(working set, peak working set)` of THIS process in bytes (Windows); `(0, 0)` where the tool cannot read it."""

    if os.name != "nt":
        return (0, 0)
    import ctypes
    from ctypes import wintypes

    class Counters(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD), ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t), ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", ctypes.c_size_t), ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]

    kernel32, psapi = ctypes.WinDLL("kernel32"), ctypes.WinDLL("psapi")
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
    counters = Counters()
    counters.cb = ctypes.sizeof(Counters)
    if not psapi.GetProcessMemoryInfo(kernel32.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
        return (0, 0)
    return (int(counters.WorkingSetSize), int(counters.PeakWorkingSetSize))


class MemorySampler:
    """The highest working set of this process seen by a sampler thread between `start` and `stop` (the OS peak cannot be reset, and the cold button's peak would hide the refreshes')."""

    def __init__(self, interval: float = 0.005) -> None:
        self.interval, self.peak, self.samples = interval, 0, 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="drag-ticks-rss", daemon=True)

    def _run(self) -> None:
        while not self._stop.wait(self.interval):
            self.peak = max(self.peak, working_set_bytes()[0])
            self.samples += 1

    def start(self) -> None:
        self.peak = working_set_bytes()[0]
        self._thread.start()

    def stop(self) -> int:
        self._stop.set()
        self._thread.join(5.0)
        self.peak = max(self.peak, working_set_bytes()[0])
        return self.peak


def install_laps(laps: Laps) -> list:
    """Wraps the lap functions in place (the product looks them up by module global at call time); returns the lap names found."""

    import importlib

    found = []
    for module_name, attribute, lap in LAPS:
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        original = getattr(module, attribute, None)
        if original is None:
            continue

        def make(function, name):
            def timed(*args, **kwargs):
                started = time.perf_counter()
                try:
                    return function(*args, **kwargs)
                finally:
                    laps.seconds[name] = laps.seconds.get(name, 0.0) + time.perf_counter() - started
                    laps.calls[name] = laps.calls.get(name, 0) + 1

            return timed

        setattr(module, attribute, make(original, lap))
        found.append(lap)
    return sorted(set(found))


def percentile(values: list, fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def stats(values: list) -> dict:
    if not values:
        return {}
    return {"n": len(values), "p50": round(percentile(values, 0.5), 5), "p90": round(percentile(values, 0.9), 5), "min": round(min(values), 5), "max": round(max(values), 5),
            "mean": round(statistics.fmean(values), 5)}


def mesh_arrays_hash(mesh) -> str:
    """Hash of what is in Blender: float32 positions and UV bytes, the polygon starts and the loop vertices (a changed layout changes it too)."""

    import numpy as np

    digest = hashlib.sha256()
    co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get("co", co)
    layer = mesh.uv_layers["UVMap"]
    uv = np.empty(len(layer.data) * 2, dtype=np.float32)
    layer.data.foreach_get("uv", uv)
    corner = np.empty(len(mesh.loops), dtype=np.int32)
    mesh.loops.foreach_get("vertex_index", corner)
    for array in (co, uv, corner):
        digest.update(array.tobytes())
    return digest.hexdigest()


class Session:
    """The scene after the cold button: what a width step needs."""

    def __init__(self, args) -> None:
        import bpy

        self.args = args
        self.bpy = bpy
        self.case = tree_ab.parse_case(args.case)
        tree_ab.load_tree(Path(args.root).resolve(), args.native_path)
        self.native = tree_ab.verify_native(args.native_path, "")
        import cftuv.envelope_production_mesh as mesh_module
        import cftuv.envelope_width_live as live
        from cftuv.envelope_alpha_preview import PreviewRequestV1
        from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

        from cftuv.envelope_alpha_preview import PreviewUnavailable

        self.mesh_module, self.live, self.request_type, self.unavailable = mesh_module, live, PreviewRequestV1, PreviewUnavailable
        scene_args = argparse.Namespace(scene_kind=self.case.scene, mesh=self.case.mesh)
        self.scene, self.obj, self.settings, self.mesh_settings = tree_ab._open_case(scene_args)
        self.settings.envelope_debug_engine = "QUEUE"
        self.settings.envelope_debug_workers = args.workers
        self.settings.envelope_debug_fan_density = self.case.density
        self.settings.envelope_debug_max_stretch = self.case.stretch
        self.settings.envelope_debug_alpha = self.case.alpha
        self.mesh_settings.kernel_backend = args.kernel_backend
        if self.case.offset >= 0:
            self.mesh_settings.offset = self.case.offset
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        self.controller = controller
        self.index = 0
        self.gc_trace = GcTrace()
        self.gc_trace.install()

    def cold_press(self) -> float:
        from cftuv.envelope_domain_pool import get_domain_pool
        from cftuv.envelope_worker_python import read_worker_python

        get_domain_pool(self.args.workers, read_worker_python())
        try:
            from cftuv.envelope_pool_prewarm import prewarm_pool

            prewarm_pool(self.args.workers, read_worker_python())
        except ImportError:
            pass
        tree_ab.select_edges(self.obj, "stored" if self.case.scene == "cover008" else "seam")
        started = time.perf_counter()
        assert self.bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
        return time.perf_counter() - started

    @property
    def decal_mesh(self):
        return self.bpy.data.objects[self.mesh_module.decal_object_name(self.obj.name)].data

    def target(self, lean, incremental: bool = False):
        target = self.live.target_of(self.settings, self.mesh_settings, self.controller.width_build)
        if incremental:
            from cftuv.envelope_production_result import REFRESH_INCREMENTAL

            target = dataclasses.replace(target, grade=REFRESH_INCREMENTAL)
        elif lean:
            from cftuv.envelope_production_result import REFRESH_LEAN

            target = dataclasses.replace(target, grade=REFRESH_LEAN)
        return target

    def shadow(self, alpha: float, applied_run) -> dict:
        """A full lean refresh at `alpha` that is never applied, and how the applied (incremental) tick differs from it. Not part of any timing."""

        self.index += 1
        request = self.request_type(self.index, float(alpha), self.target(True), time.monotonic())
        job = self.live._begin(self.controller, request)
        job.join()
        return shadow_difference(applied_run, job.result())

    def step(self, alpha: float, *, lean: bool, laps: Laps, incremental: bool = False, shadow: bool = False) -> dict:
        """One width step of the live path, call for call; the record of its times."""

        self.index += 1
        laps.reset()
        request = self.request_type(self.index, float(alpha), self.target(lean, incremental), time.monotonic())
        self.gc_trace.window_reset()
        started = time.perf_counter()
        job = self.live._begin(self.controller, request)
        begun = time.perf_counter()
        job.join()
        joined = time.perf_counter()
        live_run = job.result()
        reason = self.live._validity(self.controller, request, job)
        if reason:
            raise RuntimeError(f"the width step is not valid: {reason}")
        applying = time.perf_counter()
        outcome = "applied"
        try:
            self.live._apply(self.controller, request, job, live_run)
        except self.unavailable as exc:  # a named refusal of the apply (the scheduler records it the same way): the tick is counted with its reason
            outcome = str(exc)
        finished = time.perf_counter()
        lap_record = laps.snapshot()  # before the shadow refresh: its laps are not this tick's
        plan = getattr(live_run, "plan", None)
        record_plan = None if plan is None else {
            "mode": plan.mode, "reason": plan.reason, "served": len(plan.served or ()), "held": len(plan.held or {}), "held_reasons": dict(Counter((plan.held or {}).values())),
            "seconds": round(plan.seconds, 6), "composite": plan.composite, "fallback": plan.fallback,
        }
        shadow_record = self.shadow(alpha, live_run) if (shadow and plan is not None and plan.incremental) else None
        return {
            "plan": record_plan, "shadow": shadow_record,
            **self.gc_trace.window(started), "gc_enabled_after_step": gc.isenabled(), "started_at": started, "finished_at": finished, "rss_after_mb": round(working_set_bytes()[0] / 1048576, 1),
            "apply_outcome": outcome,
            "alpha": float(alpha), "lean": lean, "wall": finished - started, "begin_main": begun - started, "thread": float(job.seconds), "join_wait": joined - begun,
            "apply_main": finished - applying, "run_production": float(live_run.run.wall_seconds), "laps": lap_record,
            "refresh": getattr(live_run.run, "refresh", "COMMIT"), "domains": len(live_run.run.results),
            "lean_domains": sum(1 for item in live_run.run.results if getattr(item, "lean", False)),
        }


def shadow_difference(applied, exact) -> dict:
    """The incremental tick (`applied`, a `LiveRunV1`) against the full lean refresh of the same width (`exact`): per domain of the MESH SAMPLE, nothing hidden.

    The computed domains are the ones the tick's run returned (placement other than `certificate`); every other domain of the sample was served (moved by the certificate, or by the model frame in the composite).
    A served domain's structure digest is the one of the verification base, so a digest that differs from the exact one is a FLIP the tick could not see. Positions and UV are those of the sample (mesh coordinates,
    after the weld and the offset; metres / tile units) at the domain's own vertices and loops; `arrays` compares the whole mesh arrays when the layout is the same."""

    import numpy as np

    computed = {int(item.patch_id) for item in applied.run.results if item.placement != "certificate"}
    inc = {item.patch_id: item for item in applied.sample.domains}
    full = {item.patch_id: item for item in exact.sample.domains}
    flips, position_served, uv_served, position_recomputed, uv_recomputed, recomputed_different = [], 0.0, 0.0, 0.0, 0.0, []
    for patch, domain in inc.items():
        other = full.get(patch)
        if other is None:
            recomputed_different.append(patch)
            continue
        flipped = domain.structure != other.structure
        if patch not in computed and flipped:
            flips.append(patch)
        if domain.vertices.shape != other.vertices.shape or domain.loop_count != other.loop_count:
            if patch in computed:
                recomputed_different.append(patch)
            continue
        position = float(np.max(np.abs(applied.sample.positions[domain.vertices] - exact.sample.positions[other.vertices]), initial=0.0))
        uv = float(np.max(np.abs(applied.sample.uvs[domain.loop_start : domain.loop_start + domain.loop_count] - exact.sample.uvs[other.loop_start : other.loop_start + other.loop_count]), initial=0.0))
        if patch in computed:
            position_recomputed, uv_recomputed = max(position_recomputed, position), max(uv_recomputed, uv)
            if position or uv or flipped:
                recomputed_different.append(patch)
        elif not flipped:
            position_served, uv_served = max(position_served, position), max(uv_served, uv)
    same_layout = applied.sample.layout_digest == exact.sample.layout_digest
    arrays = {"same_layout": same_layout}
    if same_layout:
        arrays["position_max"] = float(np.max(np.abs(applied.sample.positions - exact.sample.positions), initial=0.0))
        arrays["uv_max"] = float(np.max(np.abs(applied.sample.uvs - exact.sample.uvs), initial=0.0))
        arrays["owners_equal"] = bool(np.array_equal(applied.sample.image.face_owner, exact.sample.image.face_owner))
    plan = getattr(applied, "plan", None)
    return {
        "served": len(inc) - len(computed & set(inc)), "recomputed": len(computed), "flipped_served": flips, "served_position_max": position_served, "served_uv_max": uv_served,
        "recomputed_position_max": position_recomputed, "recomputed_uv_max": uv_recomputed, "recomputed_different": recomputed_different, "arrays": arrays,
        "composite": None if plan is None else bool(plan.composite), "fallback": None if plan is None else plan.fallback,
    }


def reference_digest(mesh) -> str:
    """The receipt digest as `mesh_content_digest` read it before `foreach_get` (item by item through RNA), verbatim: the reference of the equality."""

    mesh_module = sys.modules["cftuv.envelope_production_mesh"]
    uv_layer = mesh.uv_layers.get(mesh_module.DECAL_UV_LAYER)
    payload = {
        "positions": [tuple(item.co) for item in mesh.vertices],
        "faces": [tuple(item.vertices) for item in mesh.polygons],
        "uvs": [] if uv_layer is None else [tuple(item.uv) for item in uv_layer.data],
        "seams": sorted(tuple(sorted(item.vertices)) for item in mesh.edges if item.use_seam),
    }
    for name in (mesh_module.DECAL_DOMAIN_ATTRIBUTE, mesh_module.DECAL_OWNER_ATTRIBUTE):
        attribute = mesh.attributes.get(name)
        payload[name] = [] if attribute is None else [item.value for item in attribute.data]
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("ascii")).hexdigest()


def prove_digest(session: Session) -> dict:
    """Both digests of the real decal mesh, equal or not, and the milliseconds of each (best of five)."""

    mesh = session.decal_mesh
    spent = {"reference": [], "foreach_get": []}
    values = {}
    for _ in range(5):
        started = time.perf_counter()
        values["reference"] = reference_digest(mesh)
        spent["reference"].append(time.perf_counter() - started)
        started = time.perf_counter()
        values["foreach_get"] = session.mesh_module.mesh_content_digest(mesh)
        spent["foreach_get"].append(time.perf_counter() - started)
    return {"equal": values["reference"] == values["foreach_get"], "digest": values["foreach_get"][:16], "vertices": len(mesh.vertices), "polygons": len(mesh.polygons),
            "loops": len(mesh.loops), "reference_ms": round(min(spent["reference"]) * 1000, 3), "foreach_get_ms": round(min(spent["foreach_get"]) * 1000, 3)}


def prove_lean(session: Session, laps: Laps, widths: list) -> list:
    """Per width: lean refresh -> mesh digest; the full path at the same width -> mesh digest. Equal digests: the lean frame is the committed mesh."""

    proofs = []
    for width in widths:
        lean_tick = session.step(width, lean=True, laps=laps)
        if lean_tick["apply_outcome"] != "applied":  # the layout of this width is not the layout on screen: nothing was written, so there is nothing to compare
            proofs.append({"width": width, "skipped": lean_tick["apply_outcome"]})
            continue
        lean_digest = session.mesh_module.mesh_content_digest(session.decal_mesh)
        lean_arrays = mesh_arrays_hash(session.decal_mesh)
        session.step(width, lean=False, laps=laps)
        full_digest = session.mesh_module.mesh_content_digest(session.decal_mesh)
        full_arrays = mesh_arrays_hash(session.decal_mesh)
        proofs.append({"width": width, "equal_mesh_digest": lean_digest == full_digest, "equal_arrays_hash": lean_arrays == full_arrays, "mesh_digest": full_digest[:16]})
    return proofs


def main() -> int:
    args = _arguments()
    report: dict = {"schema": "cftuv.drag-ticks.v1", "args": vars(args), "failure": None}
    try:
        session = Session(args)
        report["native_build_id"] = session.native["build_id"]
        laps = Laps()
        report["laps_found"] = install_laps(laps)
        lean_supported = hasattr(session.live.WidthLiveTargetV1, "grade")
        if args.grade in ("lean", "incremental") and not lean_supported:
            raise RuntimeError("this tree has no WIDTH_REFRESH_LEAN_V1 (WidthLiveTargetV1.grade)")
        if args.grade == "incremental" and not hasattr(session.live, "DRAG_GRADES"):
            raise RuntimeError("this tree has no WIDTH_REFRESH_INCREMENTAL_V1 (envelope_width_live.DRAG_GRADES)")
        report["cold_press_s"] = round(session.cold_press(), 4)
        if args.mode == "object" and session.bpy.context.mode != "OBJECT":
            session.bpy.ops.object.mode_set(mode="OBJECT")
        report["source_mode"] = session.bpy.context.mode
        if args.prove_digest:
            report["digest_proof"] = prove_digest(session)
        base = float(session.settings.envelope_debug_alpha)
        before = (session.mesh_module.mesh_content_digest(session.decal_mesh), mesh_arrays_hash(session.decal_mesh))
        step = base * args.rel
        session.controller.width_drag_active = True  # the tool's drag (lean refreshes are valid only while it runs)
        ticks, widths = [], [base + step * (index + 1) for index in range(args.ticks + args.confirms + 1)]
        if args.prove_lean:
            report["proofs"] = prove_lean(session, laps, [base * (1.0 + args.prove_rel * (index + 1)) for index in range(args.prove_lean)])
        gc_before, rss_cold = session.gc_trace.cumulative(), working_set_bytes()
        sampler = MemorySampler()
        sampler.start()
        phase_started = time.perf_counter()
        for width in widths[: args.ticks]:
            ticks.append(session.step(width, lean=args.grade in ("lean", "incremental"), laps=laps, incremental=args.grade == "incremental", shadow=args.shadow))
        ticks_finished = time.perf_counter()
        gc_ticks = session.gc_trace.cumulative()
        session.controller.width_drag_active = False
        confirms = [session.step(width, lean=False, laps=laps) for width in widths[args.ticks : args.ticks + args.confirms]]
        rss_peak = sampler.stop()
        gc_after = session.gc_trace.cumulative()
        previous_end = phase_started
        for item in ticks:  # the cycle: from the end of the previous refresh (the start of the phase for the first) to the end of this one, so a collection a pause deferred past the apply is in it
            item["cycle"] = item["finished_at"] - previous_end
            previous_end = item["finished_at"]
        report["tick_cycle"] = stats([item["cycle"] for item in ticks])
        report["ticks_phase_seconds"] = ticks_finished - phase_started
        report["gc"] = {
            "ticks_total_seconds": gc_ticks[0] - gc_before[0], "ticks_in_windows_seconds": sum(item["gc_seconds"] for item in ticks), "ticks_collections": gc_ticks[1] - gc_before[1],
            "ticks_gen2": gc_ticks[2] - gc_before[2], "drag_total_seconds": gc_after[0] - gc_before[0], "drag_collections": gc_after[1] - gc_before[1], "drag_gen2": gc_after[2] - gc_before[2],
        }
        report["gc"]["ticks_outside_windows_seconds"] = report["gc"]["ticks_total_seconds"] - report["gc"]["ticks_in_windows_seconds"]
        report["gc_enabled_after_every_step"] = all(item["gc_enabled_after_step"] for item in [*ticks, *confirms]) and gc.isenabled()
        report["gc_paused_in_tree"] = hasattr(session.live, "gc_paused_function")
        report["rss_mb"] = {"cold_end": round(rss_cold[0] / 1048576, 1), "ticks_peak": round(rss_peak / 1048576, 1), "process_peak": round(working_set_bytes()[1] / 1048576, 1),
                            "sampler_samples": sampler.samples}
        report["ticks"], report["confirms"] = ticks, confirms
        report["tick_wall"], report["confirm_wall"] = stats([item["wall"] for item in ticks]), stats([item["wall"] for item in confirms])
        if args.grade == "incremental":
            incremental = [item for item in ticks if item["plan"] and item["plan"]["mode"] == "WIDTH_REFRESH_INCREMENTAL_V1"]
            verification = [item for item in ticks if item["plan"] and item["plan"]["mode"] != "WIDTH_REFRESH_INCREMENTAL_V1"]
            report["incremental_tick_wall"], report["verification_tick_wall"] = stats([item["wall"] for item in incremental]), stats([item["wall"] for item in verification])
            report["verification_reasons"] = dict(Counter(item["plan"]["reason"] for item in verification))
            report["incremental_ticks"], report["verification_ticks"] = len(incremental), len(verification)
            report["incremental_served"] = stats([float(item["plan"]["served"]) for item in incremental])
            report["incremental_held"] = stats([float(item["plan"]["held"]) for item in incremental])
        report["tick_thread"] = stats([item["thread"] for item in ticks])
        report["tick_apply_main"] = stats([item["apply_main"] for item in ticks])
        report["tick_outcomes"] = {name: sum(1 for item in ticks if item["apply_outcome"] == name) for name in sorted({item["apply_outcome"] for item in ticks})}
        report["tick_begin_main"] = stats([item["begin_main"] for item in ticks])
        report["final_mesh_digest"] = session.mesh_module.mesh_content_digest(session.decal_mesh)
        report["final_arrays_hash"] = mesh_arrays_hash(session.decal_mesh)
        report["final_width"] = confirms[-1]["alpha"] if confirms else widths[args.ticks - 1]
        report["cold_mesh"] = {"digest": before[0], "arrays_hash": before[1]}
        laps_names = sorted({name for item in ticks for name in item["laps"]})
        report["tick_laps_p50"] = {name: round(statistics.median(item["laps"].get(name, [0, 0.0])[1] for item in ticks), 6) for name in laps_names}
        report["confirm_laps_p50"] = {name: round(statistics.median(item["laps"].get(name, [0, 0.0])[1] for item in confirms), 6) for name in sorted({n for c in confirms for n in c["laps"]})} if confirms else {}
    except Exception:  # noqa: BLE001 - the traceback is the report
        report["failure"] = traceback.format_exc()
        print(report["failure"])
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001 - the cleanup does not change the verdict
        print("pool shutdown:", type(exc).__name__, exc)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    if report["failure"] is None:
        print(f"DRAG_TICKS tick p50 {report['tick_wall'].get('p50')} p90 {report['tick_wall'].get('p90')} s; confirm {report['confirm_wall'].get('p50')} s; proofs {report.get('proofs')}")
    print("DRAG_TICKS_OK" if report["failure"] is None else "DRAG_TICKS_FAILED")
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
