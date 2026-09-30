"""Измерение PARALLEL-DOMAINS: те же домены здания, но в пуле процессов.

Маршрут одного домена побитово тот же, что в `artifacts/building_full_sweep/
sweep.py` (и, значит, в кнопке Blender): `build_envelope_analysis_snapshot` ->
`build_envelope_decal_request` -> `run_queue_domain`, alpha "0.45", штатный кап
работы. Отличие одно: кто и в каком порядке вызывает.

Воркер (spawn) один раз на процесс делает импорты, `big_scene.survey()` и
`stage_domain_inputs`; задача — один `patch_id`. Перед каждым доменом сбрасывается
память канонизации и счётчик внебюджетной работы ровно как в sweep.py — поэтому
счётчики обязаны совпасть с последовательным прогоном независимо от порядка.

Запуск из `artifacts/perf_prepare_diag` (там `env.py`):

    PYTHONPATH=. python3 ../parallel_domains_spike/pool_sweep.py \
        --workers 8 --order <baseline.json> --out <куда.json>

`--workers 0` — тот же цикл в ЭТОМ процессе, в порядке patch_id (эталон с
отпечатками ответа). `--pickle-dir` — зонд переносимости `prepared`/записи
домена (только для `--only`-списка, чтобы не мешать замеру скорости).
Воркеры никогда не импортируют настоящий `bpy`: `env` ставит пустую заглушку.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import os
import pickle
import sys
import time
from pathlib import Path

_T_MODULE = time.time()
HERE = Path(__file__).resolve().parent
DIAG = HERE.parent / "perf_prepare_diag"
ALPHA_VALUE = 0.45
ALPHA_TEXT = "0.45"

_CTX: dict = {}


def _fingerprints(domain):
    """Отпечатки ОТВЕТА домена: без секунд, только то, что считает ядро."""

    def digest(payload) -> str:
        return hashlib.sha256(repr(payload).encode("utf-8")).hexdigest()[:20]

    geometry = (domain.regions, domain.faces, domain.segments)
    meta = (
        domain.preparation_outcome,
        domain.coverage_outcome,
        domain.detail,
        domain.lattice_scale,
        domain.alpha,
        domain.lattice_alpha,
        domain.law_names,
    )
    return {
        "fp_geometry": digest(geometry),
        "fp_meta": digest(meta),
        "fp_counters": digest(domain.counters),
        "fp_host_counters": digest(domain.host_counters),
        "repr_has_address": " at 0x" in repr(geometry),
    }


def init_worker(quiet: bool = True):
    try:
        import psutil
    except ImportError:  # Blender python has no psutil: fall back to ctypes
        psutil = None

    t_begin = time.time()
    clock = time.perf_counter()
    if str(DIAG) not in sys.path:
        sys.path.insert(0, str(DIAG))
    sink = io.StringIO() if quiet else sys.stdout
    with contextlib.redirect_stdout(sink):
        import env  # noqa: F401

        from cftuv_envelope import exact_sqrt_sum as canon
        import big_scene
        from cftuv.envelope_queue_export import run_queue_domain
        from cftuv.envelope_request_export import (
            EnvelopeHostAdapterError,
            _typed_value,
            build_envelope_analysis_snapshot,
            build_envelope_decal_request,
        )
        from cftuv.envelope_topology_export import stage_domain_inputs

        imports_done = time.perf_counter()
        _, bundle, selected, _ = big_scene.survey()
        survey_done = time.perf_counter()
        _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
            bundle, selected
        )
        stage_done = time.perf_counter()

    real_bpy = getattr(sys.modules.get("bpy"), "__file__", None)
    _CTX.update(
        canon=canon,
        run_queue_domain=run_queue_domain,
        EnvelopeHostAdapterError=EnvelopeHostAdapterError,
        typed_value=_typed_value,
        build_snapshot=build_envelope_analysis_snapshot,
        build_request=build_envelope_decal_request,
        bundle=bundle,
        revision=revision,
        patch_ids=frozenset(patch_ids),
        request_id=request_id,
        by_domain=by_domain,
        psutil=psutil,
        startup={
            "pid": os.getpid(),
            "t_module_import": _T_MODULE,
            "t_init_begin": t_begin,
            "imports_seconds": round(imports_done - clock, 3),
            "survey_seconds": round(survey_done - imports_done, 3),
            "stage_seconds": round(stage_done - survey_done, 3),
            "init_seconds": round(stage_done - clock, 3),
            "t_init_done": time.time(),
            "real_bpy_file": real_bpy,
            "bpy_is_stub": real_bpy is None,
        },
    )


def _rss_ctypes():
    import ctypes
    from ctypes import wintypes

    class Counters(ctypes.Structure):
        _fields_ = [
            ("cb", wintypes.DWORD),
            ("PageFaultCount", wintypes.DWORD),
            ("PeakWorkingSetSize", ctypes.c_size_t),
            ("WorkingSetSize", ctypes.c_size_t),
            ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
            ("QuotaPagedPoolUsage", ctypes.c_size_t),
            ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
            ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
            ("PagefileUsage", ctypes.c_size_t),
            ("PeakPagefileUsage", ctypes.c_size_t),
        ]

    counters = Counters()
    counters.cb = ctypes.sizeof(Counters)
    kernel32 = ctypes.windll.kernel32
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    psapi = ctypes.windll.psapi
    psapi.GetProcessMemoryInfo.argtypes = [
        wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD,
    ]
    psapi.GetProcessMemoryInfo(
        kernel32.GetCurrentProcess(), ctypes.byref(counters), counters.cb
    )
    return counters.WorkingSetSize, counters.PeakWorkingSetSize


def _rss(ctx):
    if ctx["psutil"] is None:
        current, peak = _rss_ctypes()
    else:
        info = ctx["psutil"].Process().memory_info()
        current, peak = info.rss, getattr(info, "peak_wset", info.rss)
    return {
        "rss_mb": round(current / 2**20, 1),
        "peak_wset_mb": round(peak / 2**20, 1),
    }


def compute_domain(patch_id: int, probe: str | None = None):
    """Один домен ровно как в sweep.py. `probe` — каталог зонда pickle."""

    ctx = _CTX
    canon = ctx["canon"]
    started_epoch = time.time()
    if patch_id not in ctx["patch_ids"]:
        return {"patch_id": patch_id, "error": "patch_id not staged"}
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    canon.reset_factorization_memory()
    canon.reset_unbudgeted_work()
    started = time.perf_counter()
    prepared = domain = None
    try:
        snapshot = ctx["build_snapshot"](
            ctx["bundle"], included_patch_ids=frozenset({patch_id})
        )
        request = ctx["build_request"](
            snapshot,
            frozenset(ctx["by_domain"][domain_id]),
            ALPHA_VALUE,
            decal_request_id_value=ctx["request_id"],
            density=0,
        )
        host_export_seconds = time.perf_counter() - started
        prepared, domain = ctx["run_queue_domain"](
            patch_id, domain_id, snapshot, request, ALPHA_TEXT
        )
    except ctx["EnvelopeHostAdapterError"] as refusal:
        row = {
            "seconds": round(time.perf_counter() - started, 3),
            "outcome": "HOST_ADMISSION_REFUSED",
            "leaked_unbudgeted": canon.UNBUDGETED_WORK.spent,
            "detail": str(refusal),
        }
    else:
        seconds = time.perf_counter() - started
        named = getattr(prepared, "work_budget", None)
        charged = () if named is None else named.counters()
        if not charged:
            charged = tuple(
                (name, value)
                for name, value in prepared.counters
                if name.startswith("EXACT_WORK_")
            )
        row = dict(charged)
        row["seconds"] = round(seconds, 3)
        row["outcome"] = f"{domain.preparation_outcome}/{domain.coverage_outcome}"
        row["leaked_unbudgeted"] = canon.UNBUDGETED_WORK.spent
        row["detail"] = domain.detail
        row["host_export_seconds"] = round(host_export_seconds, 3)
        row["prepare_seconds"] = round(domain.prepare_seconds, 3)
        row["coverage_seconds"] = round(domain.coverage_seconds, 3)
        row["prepared_counters"] = {
            name: value for name, value in prepared.counters
        }
        row.update(_fingerprints(domain))
    row["patch_id"] = patch_id
    row["pid"] = os.getpid()
    row["t_start"] = started_epoch
    row["t_end"] = time.time()
    row.update(_rss(ctx))
    if not ctx.get("startup_sent"):
        ctx["startup_sent"] = True
        row["worker_startup"] = dict(ctx["startup"])
    if probe is not None and prepared is not None:
        row["pickle_probe"] = _pickle_probe(patch_id, prepared, domain, probe)
    return row


def _timed_dumps(obj):
    started = time.perf_counter()
    try:
        blob = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    except Exception as error:  # noqa: BLE001 - нас интересует сам отказ
        return None, {
            "ok": False,
            "seconds": round(time.perf_counter() - started, 4),
            "error": f"{type(error).__name__}: {str(error)[:300]}",
        }
    seconds = time.perf_counter() - started
    return blob, {
        "ok": True,
        "bytes": len(blob),
        "dumps_seconds": round(seconds, 4),
    }


def _pickle_probe(patch_id, prepared, domain, directory):
    from dataclasses import replace

    out = {}
    light = replace(domain, preparation=None)
    for name, obj in (
        ("prepared", prepared),
        ("domain_with_preparation", domain),
        ("domain_light", light),
    ):
        blob, info = _timed_dumps(obj)
        if blob is not None:
            started = time.perf_counter()
            try:
                pickle.loads(blob)
                info["loads_same_process_seconds"] = round(
                    time.perf_counter() - started, 4
                )
            except Exception as error:  # noqa: BLE001
                info["loads_same_process_error"] = (
                    f"{type(error).__name__}: {str(error)[:300]}"
                )
            Path(directory).mkdir(parents=True, exist_ok=True)
            (Path(directory) / f"patch{patch_id}.{name}.pkl").write_bytes(blob)
        out[name] = info
    # Что именно мешает: по полям `prepared`.
    fields = {}
    for slot in type(prepared).__slots__:
        _, info = _timed_dumps(getattr(prepared, slot, None))
        fields[slot] = (
            {"ok": True, "bytes": info["bytes"]} if info["ok"] else info
        )
    out["prepared_fields"] = fields
    out["expected_fp"] = _fingerprints(domain)
    return out


def _task(args):
    patch_id, probe = args
    return compute_domain(patch_id, probe)


def _load_order(path):
    rows = json.loads(Path(path).read_text(encoding="utf-8"))["domains"]
    ids = sorted(
        ((int(key[5:]), row["seconds"]) for key, row in rows.items()),
        key=lambda item: (-item[1], item[0]),
    )
    return [patch_id for patch_id, _ in ids]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--order", required=True, help="baseline.json")
    parser.add_argument("--out", required=True)
    parser.add_argument("--only", default="", help="patch ids через запятую")
    parser.add_argument("--pickle-dir", default=None)
    parser.add_argument("--passes", type=int, default=1)
    args = parser.parse_args()

    order = _load_order(args.order)
    if args.only:
        order = [int(v) for v in args.only.split(",")]
    probe = args.pickle_dir

    record = {"workers": args.workers, "passes": []}
    if args.workers == 0:
        init_worker(quiet=True)
        started = time.perf_counter()
        rows = [compute_domain(pid, probe) for pid in sorted(order)]
        wall = time.perf_counter() - started
        record["startup"] = dict(_CTX["startup"])
        record["passes"].append({"wall_seconds": round(wall, 3), "rows": rows})
    else:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(
            max_workers=args.workers, initializer=init_worker
        ) as pool:
            t_created = time.time()
            started = time.perf_counter()
            for pass_index in range(args.passes):
                pass_started = time.perf_counter()
                rows = list(pool.map(_task, [(pid, probe) for pid in order]))
                wall = time.perf_counter() - pass_started
                record["passes"].append(
                    {
                        "wall_seconds": round(wall, 3),
                        "rows": rows,
                    }
                )
            record["t_created"] = t_created
    # Стартовые замеры воркера едут в его первой строке (`worker_startup`).
    record["machine_cores"] = os.cpu_count()
    record["python"] = sys.version.split()[0]
    Path(args.out).write_text(
        json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    last = record["passes"][-1]
    print(
        f"workers={args.workers} passes={len(record['passes'])} "
        f"wall={[p['wall_seconds'] for p in record['passes']]} "
        f"domains={len(last['rows'])}"
    )


if __name__ == "__main__":
    main()
