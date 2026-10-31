"""Critical path of a pooled width press, parent AND workers (headless profiling tool; the scene is never saved).

    blender -b <frozen scene> --python-exit-code 1 --python tools/blender_worker_critpath.py -- \
        --root <tree> --native-path <site> --case cover008 --workers 12 --out out.json [--worker-cprofile]
    python tools/blender_worker_critpath.py --analyse out.json [more.json ...]

Cold press, then `--rounds` x (lean tick, COMMIT) at +1 % / +1.5 % steps (the presses of parentprof/crit2). Clocks are `time.perf_counter()`
(QueryPerformanceCounter: one clock for every process on Windows), so worker and parent stamps share one axis.

* parent: wall of run_production and build_mesh_arrays; start/end of the named parent phases; every send (`_ship`) and settle (`_Run._settle`) with the task id;
  every reply decode in the reader threads (`pickle.loads`: start, end, bytes, task id).
* workers (probe injected into the pool bootstrap before the pool starts): one JSON line per task - wait for the frame, frame bytes, decode, preparation lookup,
  produce/prepare, coverage, materializer laps, view packing, reply encode, thread CPU, and every native call (shim-level `cftuv_native.<op>` and `_core.<fn>`).
* `--worker-cprofile`: a cProfile per worker over every non-cold task (distorts the wall; for the Python-glue hotspots only), dumped at pool shutdown.
"""
from __future__ import annotations

import argparse
import functools
import json
import os
import sys
import threading
import time
import traceback
from pathlib import Path

sys.dont_write_bytecode = True
CASES = {"building": ("testscene", "building", 0.2239, "2", 42, -1.0), "cover008": ("cover008", "cover.008", 0.04214220866560936, "2", 20, 0.004999999888241291)}

WORKER_PROBE = r'''
import json as _cjson, time as _ctime, functools as _cfunctools, types as _ctypes
_CRIT_PATH = os.path.join(CRIT_DIR, "w%d.jsonl" % os.getpid())
_CRIT_HANDLE = open(_CRIT_PATH, "a", encoding="utf-8")
_CRIT = {"read0": None, "read1": None, "dec1": None, "req": 0, "native": {}, "stage": {}, "laps": [], "paths": [], "solve": 0.0, "encode": 0.0}
_CRIT_PROFILE = [None]
def _crit_add(table, name, seconds):
    row = table.get(name)
    if row is None:
        table[name] = [1, seconds]
    else:
        row[0] += 1
        row[1] += seconds
def _crit_stage(owner, name, label):
    original = getattr(owner, name)
    @_cfunctools.wraps(original)
    def wrapper(*args, **kwargs):
        t0 = _ctime.perf_counter()
        try:
            return original(*args, **kwargs)
        finally:
            _crit_add(_CRIT["stage"], label, _ctime.perf_counter() - t0)
    setattr(owner, name, wrapper)
    return original
def _crit_native_wrap(function, label):
    @_cfunctools.wraps(function)
    def wrapper(*args, **kwargs):
        t0 = _ctime.perf_counter()
        try:
            return function(*args, **kwargs)
        finally:
            _crit_add(_CRIT["native"], label, _ctime.perf_counter() - t0)
    return wrapper
def _crit_install():
    pool = importlib.import_module(spec["package"] + ".envelope_domain_pool")
    original_read_bytes = pool.read_frame_bytes
    def read_frame_bytes(stream):
        _CRIT["read0"] = _ctime.perf_counter()
        found = original_read_bytes(stream)
        _CRIT["read1"] = _ctime.perf_counter()
        _CRIT["req"] = 0 if found is None else len(found)
        return found
    pool.read_frame_bytes = read_frame_bytes
    original_decode = pool.decode_task
    def decode_task(value):
        found = original_decode(value)
        _CRIT["dec1"] = _ctime.perf_counter()
        return found
    pool.decode_task = decode_task
    original_solve = pool.solve_task
    def solve_task(task):
        t0 = _ctime.perf_counter()
        found = original_solve(task)
        _CRIT["solve"] = _ctime.perf_counter() - t0
        return found
    pool.solve_task = solve_task
    original_reply = pool._reply_frame
    def _reply_frame(task_id, result):
        t0 = _ctime.perf_counter()
        found = original_reply(task_id, result)
        _CRIT["encode"] = _ctime.perf_counter() - t0
        return found
    pool._reply_frame = _reply_frame
    _crit_stage(pool, "_packed_cold_reply", "cold_pack")
    original_serve = pool.serve_task
    def serve_task(task):
        for key in ("native", "stage"):
            _CRIT[key] = {}
        _CRIT["laps"] = []
        _CRIT["paths"] = []
        _CRIT["solve"] = _CRIT["encode"] = 0.0
        profiler = _CRIT_PROFILE[0] if task.cold is None else None
        c0 = _ctime.thread_time()
        t0 = _ctime.perf_counter()
        if profiler is not None:
            profiler.enable()
        frame = original_serve(task)
        if profiler is not None:
            profiler.disable()
        t1 = _ctime.perf_counter()
        c1 = _ctime.thread_time()
        production = getattr(task, "production", None)
        row = {"pid": os.getpid(), "task": task.task_id, "domain": getattr(task, "domain_id", ""), "patch": getattr(task, "patch_id", -1),
               "read0": _CRIT["read0"], "read1": _CRIT["read1"], "dec1": _CRIT["dec1"], "req": _CRIT["req"],
               "serve0": t0, "serve1": t1, "cpu": c1 - c0, "solve": _CRIT["solve"], "encode": _CRIT["encode"], "reply": len(frame),
               "laps": _CRIT["laps"], "paths": _CRIT["paths"], "stage": _CRIT["stage"], "native": _CRIT["native"],
               "lean": bool(getattr(production, "lean", False)), "carried": bool(getattr(production, "carried", False)),
               "kind": "cold" if task.cold is not None else ("production" if production is not None else ("export" if task.export is not None else "other"))}
        _CRIT_HANDLE.write(_cjson.dumps(row) + "\n")
        _CRIT_HANDLE.flush()
        return frame
    pool.serve_task = serve_task
    task_module = importlib.import_module(spec["package"] + ".envelope_production_task")
    for name, label in (("prepared_of", "prepared_of"), ("produce_domain", "produce_domain"), ("lean_result", "view_pack_lean"), ("deferred_result", "view_pack_deferred"),
                        ("prepare_for_production_recorded", "prepare"), ("_source_normal", "source_normal"), ("_summary_normal", "summary_normal"), ("snap_scale_notes", "snap_scale_notes")):
        if hasattr(task_module, name):
            _crit_stage(task_module, name, label)
    result_module = importlib.import_module(spec["package"] + ".envelope_production_result")
    for name in ("_placed",):
        if hasattr(result_module, name):
            _crit_stage(result_module, name, "placed")
    if hasattr(task_module, "_placed"):
        _crit_stage(task_module, "_placed", "placed_task")
    try:
        export_input = importlib.import_module(spec["package"] + ".envelope_export_input")
        if hasattr(export_input, "task_inputs"):
            _crit_stage(export_input, "task_inputs", "export_task_inputs")
    except Exception as exc:
        _CRIT_HANDLE.write(_cjson.dumps({"probe_error": "export_input " + repr(exc)}) + "\n")
    try:
        step = importlib.import_module("cftuv_envelope.materialize.step")
        original_step = step.step_domain
        def step_domain(*args, **kwargs):
            t0 = _ctime.perf_counter()
            found = original_step(*args, **kwargs)
            _crit_add(_CRIT["stage"], "step_domain", _ctime.perf_counter() - t0)
            _CRIT["paths"].append(found.path)
            return found
        step.step_domain = step_domain
        wavefront = importlib.import_module("cftuv_envelope.wavefront")
        _crit_stage(wavefront, "conveyor_coverage", "coverage")
        domain = importlib.import_module("cftuv_envelope.materialize.domain")
        original_materialize = domain.materialize_domain
        def materialize_domain(*args, **kwargs):
            t0 = _ctime.perf_counter()
            found = original_materialize(*args, **kwargs)
            _crit_add(_CRIT["stage"], "materialize_domain", _ctime.perf_counter() - t0)
            _CRIT["laps"].append([[name, round(seconds, 7)] for name, seconds in getattr(found, "timings", ())])
            return found
        domain.materialize_domain = materialize_domain
    except Exception as exc:
        _CRIT_HANDLE.write(_cjson.dumps({"probe_error": "kernel " + repr(exc)}) + "\n")
    try:
        import cftuv_native as native
        core = native._core
        builtin = {id(value): name for name, value in vars(core).items() if isinstance(value, _ctypes.BuiltinFunctionType)}
        wrapped_core = {}
        for module_name, module in list(sys.modules.items()):
            if not module_name.startswith("cftuv_native") or module is core:
                continue
            namespace = getattr(module, "__dict__", {})
            for key, value in list(namespace.items()):
                name = builtin.get(id(value))
                if name is not None:
                    if name not in wrapped_core:
                        wrapped_core[name] = _crit_native_wrap(value, "core:" + name)
                    namespace[key] = wrapped_core[name]
        for name, value in list(vars(core).items()):
            if id(value) in builtin:
                setattr(core, name, wrapped_core.get(name) or _crit_native_wrap(value, "core:" + name))
        for name in getattr(native, "__all__", ()):
            value = getattr(native, name, None)
            if isinstance(value, _ctypes.FunctionType):
                setattr(native, name, _crit_native_wrap(value, "shim:" + name))
    except Exception as exc:
        _CRIT_HANDLE.write(_cjson.dumps({"probe_error": "native " + repr(exc)}) + "\n")
    if CRIT_CPROFILE:
        import cProfile as _ccprofile
        _CRIT_PROFILE[0] = _ccprofile.Profile()
_crit_install()
'''

WORKER_EPILOGUE = r'''
if _CRIT_PROFILE[0] is not None:
    import pstats as _cpstats, io as _cio
    _cbuffer = _cio.StringIO()
    _cstats = _cpstats.Stats(_CRIT_PROFILE[0], stream=_cbuffer)
    _cstats.sort_stats("tottime").print_stats(60)
    _cstats.sort_stats("cumulative").print_stats(60)
    _cstats.dump_stats(_CRIT_PATH + ".pstats")
    with open(_CRIT_PATH + ".cprofile.txt", "w", encoding="utf-8") as _chandle:
        _chandle.write(_cbuffer.getvalue())
_CRIT_HANDLE.close()
'''

PARENT_FUNCTIONS = (
    ("cftuv.envelope_production_export", ("run_production", "_scan_and_dispatch", "_dispatch", "_dispatch_work", "settle_dispatch", "_complete_ready", "_worker_tasks", "_domain_results", "_production_run", "_record_lean_counters", "_start_byte_check")),
    ("cftuv.envelope_production_scan", ("_scan",)),
    ("cftuv.envelope_domain_pool", ("order_by_cost", "plan_first_round", "split_first_round")),
    ("cftuv.envelope_queue_pool", ("_run_tasks", "_ship_preparations")),
    ("cftuv.envelope_snap_retry", ("retry_snap_lottery",)),
    ("cftuv.envelope_production_mesh", ("build_mesh_arrays",)),
)

CALLS: list = []
DEPTH = threading.local()
SENDS: list = []
SETTLES: list = []
DECODES: list = []
LOCK = threading.Lock()
BACKGROUND: list = []


def _timed(name, function):
    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        depth = getattr(DEPTH, name, 0)
        setattr(DEPTH, name, depth + 1)
        started = time.perf_counter()
        try:
            return function(*args, **kwargs)
        finally:
            setattr(DEPTH, name, depth)
            if depth == 0:
                CALLS.append((name, started, time.perf_counter(), threading.current_thread() is threading.main_thread()))
    return wrapper


def install_parent(pool):
    import importlib
    import pickle as real_pickle

    for module_name, names in PARENT_FUNCTIONS:
        module = importlib.import_module(module_name)
        for name in names:
            original = getattr(module, name, None)
            if original is None:
                continue
            wrapped = _timed(f"{module_name.split('.')[-1]}.{name}", original)
            for other in list(sys.modules.values()):
                namespace = getattr(other, "__dict__", None)
                if not namespace or not str(getattr(other, "__name__", "")).startswith("cftuv"):
                    continue
                for key, value in list(namespace.items()):
                    if value is original:
                        namespace[key] = wrapped
    for class_name, names in (("DomainPool", ("_run_locked", "ensure_started")), ("_Run", ("execute", "__init__"))):
        owner = getattr(pool, class_name)
        for name in names:
            setattr(owner, name, _timed(f"{class_name}.{name}", owner.__dict__[name]))
    original_idle = pool.DomainPool.run_idle

    def run_idle(self, message, on_reply, **kwargs):
        steps = [0]

        def counted(index, reply):
            steps[0] += 1
            return on_reply(index, reply)

        started = time.perf_counter()
        try:
            return original_idle(self, message, counted, **kwargs)
        finally:
            BACKGROUND.append(("run_idle", started, time.perf_counter(), steps[0]))

    pool.DomainPool.run_idle = run_idle
    original_ship = pool._ship

    def _ship(worker, task, frame, counted):
        started = time.perf_counter()
        kind = original_ship(worker, task, frame, counted)
        SENDS.append((worker.index, task.task_id, started, time.perf_counter(), len(frame), kind, task.domain_id, task.patch_id, task.cold is not None))
        return kind

    pool._ship = _ship
    original_settle = pool._Run._settle

    def _settle(self, worker, lane):
        task = lane.flying[0][0] if lane.flying else None
        started = time.perf_counter()
        done = original_settle(self, worker, lane)
        SETTLES.append((worker.index, -1 if task is None else task.task_id, started, time.perf_counter()))
        return done

    pool._Run._settle = _settle

    class _Pickle:
        def __getattr__(self, name):
            return getattr(real_pickle, name)

        def loads(self, payload, *args, **kwargs):
            started = time.perf_counter()
            value = real_pickle.loads(payload, *args, **kwargs)
            DECODES.append((started, time.perf_counter(), len(payload), getattr(value, "task_id", -1), threading.get_ident()))
            return value

    pool.pickle = _Pickle()


def _open_session(T, scene_kind, mesh):
    """The frozen case opened as parentprof/crit2 open it: NATIVE backend, a fresh session controller, the stored selection, the analysis bundle."""

    import bmesh
    import bpy
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

    scene, obj, settings, mesh_settings = T._open_case(argparse.Namespace(scene_kind=scene_kind, mesh=mesh))
    mesh_settings.kernel_backend = "NATIVE"
    controller = EnvelopeDebugSessionController()
    setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    selected = T.select_edges(obj, "stored" if scene_kind == "cover008" else "seam")
    source_bm = bmesh.from_edit_mesh(obj.data)
    source_bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in source_bm.faces)
    revision = source_revision_from_bmesh(source_bm, obj, face_indices)
    key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))
    return controller, obj, settings, mesh_settings, selected, key, data_key, bundle


def main() -> int:

    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--rounds", type=int, default=6)
    parser.add_argument("--worker-cprofile", action="store_true")
    parser.add_argument("--tick-freeze", default="on", choices=("on", "off"))
    parser.add_argument("--settle", default="on", choices=("on", "off"), help="on: after the cold press wait for the background snapshot byte check (run_idle) to end, so the ticks measure the steady state")
    parser.add_argument("--byte-check", default="all", choices=("all", "product"), help="all: the full snapshot byte check (the field tools' reference); product: the button's default (timing)")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    if args.byte_check == "all":
        os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root / "tools"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    out = Path(args.out)
    crit_dir = out.with_suffix("").as_posix() + "_workers"
    Path(crit_dir).mkdir(parents=True, exist_ok=True)
    for old in Path(crit_dir).glob("*"):
        old.unlink()
    report: dict = {"case": args.case, "failure": None, "crit_dir": crit_dir, "workers": args.workers, "worker_cprofile": args.worker_cprofile}
    try:
        import cftuv.envelope_domain_pool as pool_module
        import cftuv.envelope_production_export as export_module
        import cftuv.envelope_production_mesh as mesh_module
        from cftuv.envelope_gc_pause import gc_paused as product_pause
        from cftuv.envelope_kernel_backend import kernel_backend_of

        if args.tick_freeze == "on":  # as the product's width tick (GC_FROZEN_AFTER_WIDTH_TICK_V1) and parentprof --tick-freeze on
            def gc_paused():
                return product_pause(freeze=True, valve=False)
        else:
            gc_paused = product_pause
        from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

        assert pool_module.peek_domain_pool(args.workers) is None, "the pool started before the probe"
        bootstrap = pool_module._BOOTSTRAP
        assert T.WORKER_MAIN_CALL in bootstrap
        pool_module._BOOTSTRAP = bootstrap.replace(
            T.WORKER_MAIN_CALL,
            f"CRIT_DIR = {crit_dir!r}\nCRIT_CPROFILE = {bool(args.worker_cprofile)!r}\n" + WORKER_PROBE + "\n" + T.WORKER_MAIN_CALL + "\n" + WORKER_EPILOGUE,
        )
        install_parent(pool_module)
        report["native"] = T.verify_native(args.native_path, "")
        scene_kind, mesh, alpha, density, stretch, offset = CASES[args.case]
        controller, obj, settings, mesh_settings, selected, key, data_key, bundle = _open_session(T, scene_kind, mesh)
        presses = []
        keep = []

        def press(label, width, lean):
            CALLS.clear()
            SENDS.clear()
            SETTLES.clear()
            DECODES.clear()
            c0 = time.process_time()
            t0 = time.perf_counter()
            with gc_paused():
                run = export_module.run_production(
                    controller, bundle, frozenset(selected), width, source_object_key=key, source_data_key=data_key, density=density,
                    developable_stretch_budget=envelope_stretch_budget(stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                    workers=args.workers, kernel_backend=kernel_backend_of(mesh_settings), **({"lean": True} if lean else {}),
                )
            t1 = time.perf_counter()
            c1 = time.process_time()
            with gc_paused():
                keep[:] = [run, mesh_module.build_mesh_arrays(run.results, float(mesh_settings.offset), **({"reports": False} if lean else {}))]
            t2 = time.perf_counter()
            snap = run.profile
            counters = {}
            for item in getattr(snap, "counters", None) or ():
                counters[str(item.name)] = counters.get(str(item.name), 0) + (item.value if isinstance(item.value, (int, float)) else 0)
            totals = dict(getattr(snap, "stage_totals", None) or {})
            placements = {}
            for item in run.results:
                placements[str(getattr(item, "placement", ""))] = placements.get(str(getattr(item, "placement", "")), 0) + 1
            row = {
                "label": label, "lean": lean, "width": width, "t0": t0, "t1": t1, "t2": t2, "run": t1 - t0, "arrays": t2 - t1, "parent_process_cpu": c1 - c0,
                "domains": len(run.results), "placements": placements, "refresh": str(getattr(run, "refresh", "")),
                "calls": [(n, a, b, m) for n, a, b, m in CALLS], "sends": list(SENDS), "settles": list(SETTLES), "decodes": list(DECODES),
                "counters": {k: v for k, v in counters.items() if isinstance(v, (int, float))},
                "stage_totals": {k: float(v) for k, v in totals.items() if isinstance(v, (int, float))},
                "result_seconds": [(item.domain_id, int(item.patch_id), float(getattr(item, "seconds", 0.0) or 0.0), str(getattr(item, "step_path", ""))) for item in run.results],
            }
            presses.append(row)
            print("CRIT", label, round(row["run"], 3), round(row["arrays"], 3), len(SENDS), flush=True)

        press("cold", alpha, False)
        if args.settle == "on":
            waited = time.perf_counter()
            while not BACKGROUND and time.perf_counter() - waited < 60.0:
                time.sleep(0.1)
            report["settle_seconds"] = time.perf_counter() - waited
            print("SETTLED", round(report["settle_seconds"], 2), BACKGROUND, flush=True)
        for cycle in range(1, args.rounds + 1):
            press(f"tick{cycle}", alpha * (1 + 0.01 * cycle), True)
            press(f"commit{cycle}", alpha * (1 + 0.01 * cycle + 0.005), False)
        report["presses"] = presses
        report["background"] = list(BACKGROUND)
        pool_module.shutdown_domain_pool()  # the workers run their epilogue (cProfile dump, file close) on the end of stdin
        time.sleep(3.0 if args.worker_cprofile else 0.5)
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    out.write_text(json.dumps(report, default=str), encoding="utf-8")
    print("CRIT_OK" if report["failure"] is None else "CRIT_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


# --------------------------------------------------------------------------
# Analysis (plain Python, outside Blender)
# --------------------------------------------------------------------------


def _median(values):
    values = sorted(values)
    if not values:
        return float("nan")
    middle = len(values) // 2
    return values[middle] if len(values) % 2 else 0.5 * (values[middle - 1] + values[middle])


def _quantile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    return values[min(len(values) - 1, int(q * (len(values) - 1) + 0.5))]


def _worker_rows(crit_dir):
    rows = []
    for path in sorted(Path(crit_dir).glob("w*.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                row = json.loads(line)
                if "probe_error" in row:
                    print("PROBE ERROR", path.name, row["probe_error"])
                    continue
                rows.append(row)
    return rows


STAGE_OF_CORE = {
    "coverage": "coverage", "clip": "clip", "skeleton": "skeleton", "effective_alpha": "effective_alpha", "snapshot_check": "snapshot_check",
    "exact_mesh": "materialize_w1", "canonical_triangles": "materialize_w1", "apply_silhouette": "materialize_hot", "source_lift": "materialize_hot", "lift_vertices": "materialize_hot",
    "assemble": "assemble", "canonical_json": "digest", "instantiate": "template", "key_slopes": "certify", "frame": "frame", "density": "density", "angular": "angular", "chain_station": "chain_station",
}


def analyse_press(press, rows_by_task, worker_count):
    t0, t1 = press["t0"], press["t1"]
    rows = [rows_by_task[(press["label"], task)] for task in sorted({send[1] for send in press["sends"]}) if (press["label"], task) in rows_by_task]
    phases = {}
    for name, a, b, _main in press["calls"]:
        phases.setdefault(name, []).append((a - t0, b - t0))
    first_send = min((send[2] for send in press["sends"]), default=t1) - t0
    last_send = max((send[3] for send in press["sends"]), default=t1) - t0
    last_settle = max((settle[3] for settle in press["settles"]), default=t1) - t0
    pids = sorted({row["pid"] for row in rows})
    busy = {pid: sum(row["serve1"] - row["serve0"] for row in rows if row["pid"] == pid) for pid in pids}
    first_start = {pid: min(row["serve0"] for row in rows if row["pid"] == pid) - t0 for pid in pids}
    last_end = {pid: max(row["serve1"] for row in rows if row["pid"] == pid) - t0 for pid in pids}
    serve = [row["serve1"] - row["serve0"] for row in rows]
    stage = {}
    native = {}
    laps = {}
    paths = {}
    for row in rows:
        for name, (count, seconds) in row["stage"].items():
            entry = stage.setdefault(name, [0, 0.0])
            entry[0] += count
            entry[1] += seconds
        for name, (count, seconds) in row["native"].items():
            entry = native.setdefault(name, [0, 0.0])
            entry[0] += count
            entry[1] += seconds
        for lap_set in row["laps"]:
            for name, seconds in lap_set:
                laps[name] = laps.get(name, 0.0) + seconds
        for path in row["paths"]:
            key = path.split(":")[0] if path.startswith("FALLBACK") is False else path
            paths[key] = paths.get(key, 0) + 1
    core_total = sum(seconds for name, (count, seconds) in native.items() if name.startswith("core:"))
    shim_total = sum(seconds for name, (count, seconds) in native.items() if name.startswith("shim:"))
    # Transfer: parent send end -> worker frame decoded; reply: worker serve end -> parent decode start.
    send_of = {send[1]: send for send in press["sends"]}
    decode_of = {}
    for decode in press["decodes"]:
        decode_of.setdefault(decode[3], decode)
    settle_of = {settle[1]: settle for settle in press["settles"]}
    to_worker, queue_wait, idle_wait, reply_transfer, decode, apply = [], [], [], [], [], []
    for row in rows:
        send = send_of.get(row["task"])
        if send and row["read1"] and row["dec1"]:
            to_worker.append(row["dec1"] - send[3])
            queue_wait.append(row["serve0"] - send[3])  # sent -> worker starts it (pipe + waiting behind the lane's earlier task)
            idle_wait.append(max(0.0, row["read1"] - max(row["read0"], send[2])))
        dec = decode_of.get(row["task"])
        if dec:
            reply_transfer.append(dec[0] - row["serve1"])
            decode.append(dec[1] - dec[0])
            settle = settle_of.get(row["task"])
            if settle:
                apply.append(settle[3] - dec[1])
    # The main pool run (the first `_Run.execute`) against the snap-retry run(s) after it: per worker - late start, internal gaps, early finish.
    executes = sorted(spans for name, a, b, _main in press["calls"] if name == "_Run.execute" for spans in [(a, b)])
    lanes = {}
    if executes:
        e0, e1 = executes[0]
        main_rows = sorted((row for row in rows if e0 <= row["serve0"] <= e1), key=lambda row: row["serve0"])
        for pid in sorted({row["pid"] for row in main_rows}):
            mine = [row for row in main_rows if row["pid"] == pid]
            gaps = sum(b["serve0"] - a["serve1"] for a, b in zip(mine, mine[1:]))
            lanes[pid] = {"tasks": len(mine), "busy": sum(row["serve1"] - row["serve0"] for row in mine), "late_start": mine[0]["serve0"] - e0, "gaps": gaps,
                          "early_finish": e1 - mine[-1]["serve1"], "write_read": sum(b["read1"] - a["serve1"] for a, b in zip(mine, mine[1:]) if b["read1"]),
                          "read_wait": sum(max(0.0, b["read1"] - b["read0"]) for b in mine[1:] if b["read1"] and b["read0"])}
        main_summary = {"execute": e1 - e0, "retry_executes": [b - a for a, b in executes[1:]], "tasks": len(main_rows),
                        "work": sum(row["serve1"] - row["serve0"] for row in main_rows),
                        "late_start_mean": _median([lane["late_start"] for lane in lanes.values()]),
                        "gaps_mean": sum(lane["gaps"] for lane in lanes.values()) / max(1, len(lanes)),
                        "early_finish_mean": sum(lane["early_finish"] for lane in lanes.values()) / max(1, len(lanes)),
                        "busy_mean": sum(lane["busy"] for lane in lanes.values()) / max(1, len(lanes)),
                        "write_read_mean": sum(lane["write_read"] for lane in lanes.values()) / max(1, len(lanes)),
                        "read_wait_mean": sum(lane["read_wait"] for lane in lanes.values()) / max(1, len(lanes))}
    else:
        main_summary = {}
    tail = max(rows, key=lambda row: row["serve1"] - row["serve0"]) if rows else None
    makespan = (max(last_end.values()) - min(first_start.values())) if rows else 0.0
    work = sum(serve)
    ideal = work / max(1, worker_count)
    idle_while_tail = 0.0
    if rows:
        end = max(last_end.values())
        idle_while_tail = sum(end - last_end[pid] for pid in pids)
    by_kind = {}
    for row in rows:
        kind = row["kind"] + (":lean" if row["lean"] else "")
        by_kind[kind] = by_kind.get(kind, 0) + 1
    return {
        "label": press["label"], "run": press["run"], "arrays": press["arrays"], "tasks": len(press["sends"]), "rows": len(rows), "domains": press["domains"], "by_kind": by_kind,
        "first_send": first_send, "last_send": last_send, "last_settle": last_settle, "after_last_settle": press["run"] - last_settle,
        "first_worker_start": min(first_start.values()) if rows else None, "last_worker_end": max(last_end.values()) if rows else None, "makespan": makespan,
        "work": work, "ideal_makespan": ideal, "busy_per_worker": sorted(busy.values()), "idle_while_tail": idle_while_tail, "cpu_sum": sum(row["cpu"] for row in rows),
        "serve_p50": _quantile(serve, 0.5), "serve_p90": _quantile(serve, 0.9), "serve_p99": _quantile(serve, 0.99), "serve_max": max(serve) if serve else 0.0,
        "tail": None if tail is None else {"patch": tail["patch"], "domain": tail["domain"], "seconds": tail["serve1"] - tail["serve0"], "start": tail["serve0"] - t0, "end": tail["serve1"] - t0,
                                          "stage": tail["stage"], "laps": tail["laps"], "native": tail["native"], "req": tail["req"], "reply": tail["reply"], "paths": tail["paths"]},
        "top10": [{"patch": row["patch"], "seconds": round(row["serve1"] - row["serve0"], 4), "start": round(row["serve0"] - t0, 4), "pid": row["pid"]} for row in sorted(rows, key=lambda r: -(r["serve1"] - r["serve0"]))[:10]],
        "main_run": main_summary, "lanes": lanes, "serve_all": sorted(serve),
        "stage": stage, "native": native, "laps": laps, "paths": paths, "core_total": core_total, "shim_total": shim_total,
        "solve_sum": sum(row["solve"] for row in rows), "encode_sum": sum(row["encode"] for row in rows), "decode_in_worker_sum": sum((row["serve0"] - row["dec1"]) for row in rows if row["dec1"]),
        "worker_frame_decode_sum": sum((row["dec1"] - row["read1"]) for row in rows if row["dec1"] and row["read1"]),
        "req_bytes": sum(row["req"] for row in rows), "reply_bytes": sum(row["reply"] for row in rows),
        "to_worker_p50": _median(to_worker), "queue_wait_p50": _median(queue_wait), "queue_wait_sum": sum(queue_wait), "idle_wait_sum": sum(idle_wait),
        "reply_transfer_p50": _median(reply_transfer), "reply_transfer_max": max(reply_transfer) if reply_transfer else 0.0, "decode_sum": sum(decode), "apply_p50": _median(apply),
        "phases": {name: [round(a, 4) for a in spans[0]] + [len(spans), round(sum(b - a for a, b in spans), 4)] for name, spans in phases.items()},
        "counters": {k: v for k, v in press["counters"].items() if any(s in k for s in ("POOL", "LEAN", "INCREMENTAL", "REUSE", "BUILDS", "CACHE", "HIT", "SHIP", "SCAN"))},
    }


def analyse(paths):
    for path in paths:
        report = json.loads(Path(path).read_text(encoding="utf-8"))
        if report.get("failure"):
            print(path, "FAILED", report["failure"][-400:])
            continue
        rows = _worker_rows(report["crit_dir"])
        # A worker row belongs to the press whose window holds its serve start.
        windows = [(press["label"], press["t0"], press["t1"]) for press in report["presses"]]
        rows_by_task = {}
        for row in rows:
            for label, a, b in windows:
                if a <= row["serve0"] <= b:
                    rows_by_task[(label, row["task"])] = row
                    break
        summary = [analyse_press(press, rows_by_task, report["workers"]) for press in report["presses"]]
        out = Path(str(Path(path).with_suffix("")) + "_analysis.json")
        out.write_text(json.dumps(summary, indent=1, default=str), encoding="utf-8")
        print(f"== {path}: workers {report['workers']} -> {out}")
        for item in summary:
            print(f"{item['label']:8} run={item['run']:.3f} arr={item['arrays']:.3f} tasks={item['tasks']} first_send={item['first_send']:.3f} first_start={item['first_worker_start'] or 0:.3f} "
                  f"last_end={item['last_worker_end'] or 0:.3f} last_settle={item['last_settle']:.3f} after={item['after_last_settle']:.3f} work={item['work']:.3f} ideal={item['ideal_makespan']:.3f} "
                  f"cpu={item['cpu_sum']:.3f} core={item['core_total']:.3f} shim={item['shim_total']:.3f} p50={item['serve_p50']*1e3:.2f}ms p90={item['serve_p90']*1e3:.2f}ms max={item['serve_max']*1e3:.1f}ms "
                  f"tail={item['tail'] and item['tail']['patch']}")


if __name__ == "__main__":
    if "--analyse" in sys.argv:
        analyse([item for item in sys.argv[sys.argv.index("--analyse") + 1:]])
    else:
        code = main()
        if code:
            raise SystemExit(code)
