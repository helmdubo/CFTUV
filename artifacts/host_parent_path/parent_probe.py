"""Родительский путь кнопки и шага ширины на замороженной сцене: секунды по ступеням, цена разбора ответов и дайджесты.

Фоновый Blender БЕЗ `--factory-startup`, БЕЗ интерфейса, ничего не сохраняется (замороженная копия сцены, меш источника не правится):

    set PYTHONSAFEPATH=1
    blender -b <замороженная сцена> --python-exit-code 1 --python artifacts/host_parent_path/parent_probe.py -- \\
        --case cover|building --workers 8 --steps cold,warm,warm,warm --out probe.json [--root <дерево>] [--prewarm] [--start-scale 1.02]

`--case cover` — `cover.008` из `buildings2_2.blend` (политика и выделение сверяются с `tools/blender_field_case_cover008.CASE`),
`--case building` — меш `building` из `testscene.blend` (рёбра шва, alpha 0.2239, плотность 2, растяжение 42). `--steps`: `cold` — кнопка
(прогон + запись меша), `warm` — шаг ширины (прогон на следующей ширине, меша нет). `--prewarm` — перед кнопкой пул поднимает
`prewarm_pool` (сколько стоит первое нажатие, если пул уже стоит).

Что печатается на шаг: секунды прогона, ступени родителя (скан, рассылка, `_adopt_cold`, регистрация содержимого, разбор ответов
воркеров, проверка снапшотов, старт пула), счётчики пула и памяти шага, сумма секунд доменов и дайджест ответа (исход и дайджест содержания
каждого домена; на `cold` ещё дайджест меша). Дайджесты двух деревьев сравниваются целиком, секунды — нет.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
import threading
import time
import traceback
from pathlib import Path

import bmesh
import bpy

ROOT = Path(__file__).resolve().parents[2]
COVER_TOOL = "tools/blender_field_case_cover008.py"
BUILDING_ALPHA = 0.2239
STEP = 1.02


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--case", choices=("cover", "building"), default="cover")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--steps", default="cold,warm,warm,warm")
    parser.add_argument("--out", default="")
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument("--start-scale", type=float, default=1.0, help="множитель ширины первого шага (холодное нажатие на ширине тёплого шага соседнего прогона)")
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


TASKS: list = []


def _workers_rss_mb() -> dict:
    """Рабочий набор воркеров общего пула (Windows, `GetProcessMemoryInfo`): сумма и наибольший, МБ; пусто, если пула нет или это не Windows."""

    try:
        import ctypes
        from ctypes import wintypes

        from cftuv import envelope_domain_pool as pool

        class Counters(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD), ("PeakWorkingSetSize", ctypes.c_size_t),
                        ("WorkingSetSize", ctypes.c_size_t), ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]

        sizes = []
        for worker in pool._POOL._workers:  # noqa: SLF001 - замер
            handle = ctypes.windll.kernel32.OpenProcess(0x1000 | 0x0400, False, worker.process.pid)
            counters = Counters()
            counters.cb = ctypes.sizeof(Counters)
            if ctypes.windll.psapi.GetProcessMemoryInfo(handle, ctypes.byref(counters), counters.cb):
                sizes.append(counters.PeakWorkingSetSize / 1048576)
            ctypes.windll.kernel32.CloseHandle(handle)
        return {"workers": len(sizes), "peak_sum_mb": round(sum(sizes)), "peak_max_mb": round(max(sizes))} if sizes else {}
    except Exception:  # noqa: BLE001 - замер памяти не обязан работать везде
        return {}


def _timeline() -> dict:
    """Загрузка воркеров прогона по записям `_exchange`: занятость, хвост и самые долгие задачи."""

    rows = list(TASKS)
    TASKS.clear()
    if not rows:
        return {}
    begin = min(item[2] for item in rows)
    end = max(item[3] for item in rows)
    busy: dict = {}
    last: dict = {}
    for worker, _task, t0, t1 in rows:
        busy[worker] = busy.get(worker, 0.0) + (t1 - t0)
        last[worker] = max(last.get(worker, 0.0), t1 - begin)
    longest = sorted(((round(t1 - t0, 3), task) for _worker, task, t0, t1 in rows), reverse=True)[:8]
    return {"tasks": len(rows), "span": round(end - begin, 3), "busy_sum": round(sum(busy.values()), 3),
            "busy_by_worker": {str(k): round(v, 2) for k, v in sorted(busy.items())},
            "last_finish": {str(k): round(v, 2) for k, v in sorted(last.items())}, "longest": longest}


class Ledger:
    """Секунды и вызовы подменённых ступеней; поток-читатель родителя пишет в тот же журнал (замок)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.rows: dict[str, list] = {}

    def add(self, name: str, seconds: float, cpu: float = 0.0) -> None:
        with self.lock:
            row = self.rows.setdefault(name, [0, 0.0, 0.0])
            row[0] += 1
            row[1] += seconds
            row[2] += cpu

    def take(self) -> dict:
        with self.lock:
            taken = {name: {"calls": row[0], "seconds": round(row[1], 3), "thread_cpu": round(row[2], 3)} for name, row in self.rows.items()}
            self.rows = {}
        return taken


def _timed(ledger: Ledger, name: str, function):
    def wrapper(*args, **kwargs):
        started, cpu = time.perf_counter(), time.thread_time()
        try:
            return function(*args, **kwargs)
        finally:
            ledger.add(name, time.perf_counter() - started, time.thread_time() - cpu)

    wrapper.__wrapped__ = function
    return wrapper


def _instrument(ledger: Ledger) -> None:
    import cftuv.envelope_domain_pool as pool
    import cftuv.envelope_production_export as export
    import cftuv_envelope

    for name in ("_scan", "_dispatch", "_adopt_cold", "_register_content", "_inputs_of", "_complete_ready", "retry_snap_lottery"):
        if hasattr(export, name):
            setattr(export, name, _timed(ledger, name, getattr(export, name)))
    import cftuv.envelope_queue_pool as queue_pool

    for name in ("_ship_preparations", "_run_tasks"):
        setattr(queue_pool, name, _timed(ledger, name, getattr(queue_pool, name)))
    export._worker_tasks = _timed(ledger, "_worker_tasks", export._worker_tasks)
    export._production_input = _timed(ledger, "_production_input", export._production_input)
    export._finish_ready = _timed(ledger, "_finish_ready", export._finish_ready)
    queue_pool.PreparationBlobsV1._item_of = _timed(ledger, "blobs_item_of", queue_pool.PreparationBlobsV1._item_of)
    queue_pool.PreparationBlobsV1.key_of = _timed(ledger, "blobs_key_of", queue_pool.PreparationBlobsV1.key_of)
    pool.DomainPool.run = _timed(ledger, "pool_run", pool.DomainPool.run)
    pool.order_by_cost = _timed(ledger, "order_by_cost", pool.order_by_cost)
    original_exchange = pool._exchange

    def exchange(worker, task, frame, counted):
        started = time.perf_counter()
        try:
            return original_exchange(worker, task, frame, counted)
        finally:
            TASKS.append((worker.index, task.task_id, started, time.perf_counter()))

    pool._exchange = exchange
    if hasattr(pool, "_received"):
        pool._received = _timed(ledger, "_received", pool._received)
    pool.DomainPool.ensure_started = _timed(ledger, "pool_start", pool.DomainPool.ensure_started)
    cftuv_envelope.validate_analysis_snapshot = _timed(ledger, "validate_analysis_snapshot", cftuv_envelope.validate_analysis_snapshot)


def _cover_module():
    path = ROOT / COVER_TOOL
    spec = importlib.util.spec_from_file_location("cover_case_tool", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _select_seams(obj) -> list:
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
    chosen = []
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
        if edge.seam:
            chosen.append(edge.index)
    bmesh.update_edit_mesh(obj.data)
    return chosen


def _answer_digest(run) -> str:
    rows = sorted((item.patch_id, str(item.outcome), str(item.content_digest or ""), str(item.detail or "")[:200]) for item in run.results)
    return hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()


def _counters(run) -> dict:
    wanted = (
        "SCAN_RECORDS_REUSED", "STAGE_INPUTS_MEMO_HIT", "POOL_UNPICKLE_CPU_US", "POOL_UNPICKLE_WALL_US", "POOL_BYTES_SENT",
        "POOL_BYTES_RECEIVED", "POOL_BLOB_HITS", "POOL_BLOBS_SHIPPED", "POOL_BLOB_MISSES", "DOMAINS_WALL", "COLD_FILL",
        "PREPARATION_REUSED", "PREPARATION_BUILDS", "RESULT_CACHE_HIT", "RESULT_CACHE_MISS", "MATERIALIZED", "REFUSED",
    )
    out = {}
    for item in run.profile.counters:
        if item.patch_domain_id is None and any(item.name.endswith(name) for name in wanted):
            out[item.name] = item.value
    return out


def main() -> int:
    args = _arguments()
    root = Path(args.root).resolve()
    global ROOT
    ROOT = root
    _load_tree(root)
    ledger = Ledger()
    _instrument(ledger)
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_domain_pool import get_domain_pool, shutdown_domain_pool
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_production_export import run_production
    from cftuv.envelope_production_mesh import build_mesh_arrays, mesh_content_digest, write_decal_object
    from cftuv.envelope_production_operator import _runtime_key, _session
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget
    from cftuv.envelope_worker_python import read_worker_python

    if args.case == "cover":
        cover = _cover_module()
        obj, selected, settings, mesh_settings = cover._enter_case(bpy, bmesh)
        alpha0 = float(settings.envelope_debug_alpha)
        density = settings.envelope_debug_fan_density
        stretch = envelope_stretch_budget(settings.envelope_debug_max_stretch)
    else:
        obj = bpy.data.objects["building"]
        selected = _select_seams(obj)
        settings, mesh_settings = bpy.context.scene.hotspotuv_settings, bpy.context.scene.hotspotuv_decal_mesh
        alpha0, density = BUILDING_ALPHA, "2"
        stretch = envelope_stretch_budget(42)
    slide = envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance)
    backend = kernel_backend_of(mesh_settings)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in bm.faces)
    controller = _session(bpy.context)
    okey, dkey = _runtime_key(obj), _runtime_key(obj.data)
    revision = source_revision_from_bmesh(bm, obj, face_indices)
    bundle = controller.get_analysis_bundle(okey, dkey, revision, lambda: build_analysis_bundle(bm, face_indices, obj))

    report = {"root": str(root), "case": args.case, "workers": args.workers, "backend": str(backend), "steps": []}
    if args.prewarm:
        started = time.perf_counter()
        from cftuv.envelope_pool_prewarm import prewarm_pool

        report["prewarm_seconds"] = round(prewarm_pool(args.workers, read_worker_python()), 3)
        report["prewarm_wall"] = round(time.perf_counter() - started, 3)
        ledger.take()
    alpha = alpha0 * args.start_scale
    try:
        for index, step in enumerate(args.steps.split(",")):
            if step == "warm":
                alpha = alpha * STEP
            started = time.perf_counter()
            run = run_production(
                controller, bundle, frozenset(selected), alpha, source_object_key=okey, source_data_key=dkey, density=density,
                developable_stretch_budget=stretch, silhouette_uv_slide=slide, workers=args.workers, kernel_backend=backend,
            )
            run_seconds = time.perf_counter() - started
            row = {"step": step, "alpha": alpha, "run_seconds": round(run_seconds, 3), "domains": len(run.results),
                   "materialized": sum(1 for item in run.results if item.is_materialized), "answer_digest": _answer_digest(run),
                   "domain_seconds_sum": round(sum(item.seconds for item in run.results), 3), "counters": _counters(run)}
            if step == "cold":
                arrays = build_mesh_arrays(run.results, float(mesh_settings.offset))
                receipt = write_decal_object(obj, run.results, offset=float(mesh_settings.offset), material_name="CFTUV_Decal", width=alpha, arrays=arrays)
                row["button_seconds"] = round(time.perf_counter() - started, 3)
                row["mesh_digest"] = mesh_content_digest(bpy.data.objects[obj.name + ".CFTUV_Decal"].data)
                row["status"] = str(getattr(receipt, "status", ""))
            row["ledger"] = ledger.take()
            row["timeline"] = _timeline()
            row["workers_rss"] = _workers_rss_mb()
            report["steps"].append(row)
            print(json.dumps({key: row[key] for key in ("step", "run_seconds", "materialized", "answer_digest")}), flush=True)
    except Exception:  # noqa: BLE001 - причина идёт в отчёт
        report["failure"] = traceback.format_exc()
        print(report["failure"])
    shutdown_domain_pool()
    if args.out:
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print("PARENT_PROBE_DONE" if "failure" not in report else "PARENT_PROBE_FAILED")
    return 0 if "failure" not in report else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
