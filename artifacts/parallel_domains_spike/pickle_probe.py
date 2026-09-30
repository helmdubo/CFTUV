"""Почему `ConveyorPreparationV1` не пикулится и можно ли её донести до хоста.

    PYTHONPATH=. python3 ../parallel_domains_spike/pickle_probe.py produce <patch_id> <dir>
    PYTHONPATH=. python3 ../parallel_domains_spike/pickle_probe.py consume <patch_id> <dir>

`produce` (процесс-воркер): считает домен, снимает ОТПЕЧАТКИ ответа при alpha 0.45 и
0.30 (0.30 — путь ползунка: `conveyor_coverage(prepared, alpha)` на готовой
подготовке), ищет, какие поля `GeometryContext` не пикулятся и какой объект внутри
виноват, затем собирает КОПИЮ подготовки с опустошёнными кэшами контекста и пробует
её сериализовать. Продакшн-код не меняется: копия строится `dataclasses.replace`.

`consume` (ЧИСТЫЙ процесс — другой интерпретатор, другие таблицы интернирования):
грузит копию и считает покрытие при тех же alpha; сравнивает отпечатки с эталоном.
"""

from __future__ import annotations

import dataclasses
import json
import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import pool_sweep  # noqa: E402


def _try(obj):
    try:
        return len(pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)), None
    except Exception as error:  # noqa: BLE001
        return None, f"{type(error).__name__}: {str(error)[:160]}"


def _find_offender(obj, path, depth=0, limit=3):
    """Спуск по dict/tuple/list/dataclass до минимального непикулящегося объекта."""

    size, error = _try(obj)
    if error is None:
        return []
    found = []
    children = []
    if isinstance(obj, dict):
        children = [(f"{path}[{key!r}]"[:80], value) for key, value in obj.items()]
    elif isinstance(obj, (tuple, list)):
        children = [(f"{path}[{i}]", value) for i, value in enumerate(obj)]
    elif dataclasses.is_dataclass(obj):
        children = [
            (f"{path}.{f.name}", getattr(obj, f.name)) for f in dataclasses.fields(obj)
        ]
    for child_path, child in children:
        if _try(child)[1] is not None and depth < 8:
            found.extend(_find_offender(child, child_path, depth + 1, limit))
            if len(found) >= limit:
                return found[:limit]
    if not found:
        found.append({"path": path, "type": type(obj).__name__, "error": error})
    return found[:limit]


def _answers(ctx, prepared, patch_id, domain_id):
    from cftuv.envelope_queue_export import build_queue_domain
    from cftuv_envelope.wavefront import conveyor_coverage

    out = {}
    for alpha in ("0.45", "0.30"):
        started = time.perf_counter()
        coverage = conveyor_coverage(prepared, alpha)
        seconds = time.perf_counter() - started
        domain = build_queue_domain(
            patch_id, domain_id, prepared, coverage,
            prepare_seconds=0.0, coverage_seconds=seconds,
        )
        out[alpha] = {
            "coverage_outcome": coverage.outcome.value,
            "seconds": round(seconds, 3),
            **pool_sweep._fingerprints(domain),
        }
    return out


def produce(patch_id, directory):
    pool_sweep.init_worker(quiet=True)
    ctx = pool_sweep._CTX
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    ctx["canon"].reset_factorization_memory()
    snapshot = ctx["build_snapshot"](
        ctx["bundle"], included_patch_ids=frozenset({patch_id})
    )
    request = ctx["build_request"](
        snapshot, frozenset(ctx["by_domain"][domain_id]), 0.45,
        decal_request_id_value=ctx["request_id"], density=0,
    )
    prepared, domain = ctx["run_queue_domain"](
        patch_id, domain_id, snapshot, request, "0.45"
    )
    expected = _answers(ctx, prepared, patch_id, domain_id)
    report = {"patch_id": patch_id, "expected": expected}
    context = prepared.context
    fields = {}
    failing = []
    for field in dataclasses.fields(context):
        size, error = _try(getattr(context, field.name))
        fields[field.name] = {"bytes": size} if error is None else {"error": error}
        if error is not None:
            failing.append(field.name)
    report["context_fields"] = fields
    report["offenders"] = {
        name: _find_offender(getattr(context, name), name) for name in failing
    }
    # Копия подготовки: виноват ТОЛЬКО мутируемый кэш `metric._density_exact_memo`
    # (`intervals` держит `mpmath.ctx_iv.ivmpf`, класс которого создан динамически
    # и не пикулится). Кэш по объявлению умирает вместе с транзакцией, поэтому в
    # копии он заменён пустым экземпляром; сама метрика (gram и др.) едет как есть.
    from cftuv_envelope.reference.metric import _DensityExactMemo

    blanked = {}
    if failing == ["metric"]:
        blanked["metric"] = dataclasses.replace(
            context.metric, _density_exact_memo=_DensityExactMemo()
        )
    else:
        for name in failing:
            blanked[name] = None
    memo = context.metric._density_exact_memo
    report["density_exact_memo_entries"] = {
        slot: len(getattr(memo, slot)) for slot in memo.__slots__
    }
    stripped_context = dataclasses.replace(context, **blanked)
    stripped = dataclasses.replace(prepared, context=stripped_context)
    blob_started = time.perf_counter()
    size, error = _try(stripped)
    report["stripped"] = {
        "blanked_fields": sorted(blanked),
        "replaced": "metric._density_exact_memo -> empty _DensityExactMemo" if failing == ["metric"] else "whole fields set to None",
        "ok": error is None,
        "bytes": size,
        "error": error,
        "dumps_seconds": round(time.perf_counter() - blob_started, 4),
    }
    Path(directory).mkdir(parents=True, exist_ok=True)
    if error is None:
        (Path(directory) / f"patch{patch_id}.stripped_prepared.pkl").write_bytes(
            pickle.dumps(stripped, protocol=pickle.HIGHEST_PROTOCOL)
        )
    (Path(directory) / f"patch{patch_id}.probe.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print(json.dumps({k: report[k] for k in ("stripped", "offenders")},
                     ensure_ascii=False, indent=1)[:2500])


def consume(patch_id, directory):
    pool_sweep.init_worker(quiet=True)
    ctx = pool_sweep._CTX
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    report = json.loads(
        (Path(directory) / f"patch{patch_id}.probe.json").read_text(encoding="utf-8")
    )
    blob = (Path(directory) / f"patch{patch_id}.stripped_prepared.pkl").read_bytes()
    started = time.perf_counter()
    prepared = pickle.loads(blob)
    loads_seconds = time.perf_counter() - started
    started = time.perf_counter()
    pickle.loads(blob)
    second_loads = time.perf_counter() - started
    verdict = {
        "patch_id": patch_id,
        "bytes": len(blob),
        "loads_seconds_first_in_fresh_process": round(loads_seconds, 4),
        "loads_seconds_second": round(second_loads, 4),
    }
    try:
        got = _answers(ctx, prepared, patch_id, domain_id)
    except Exception as error:  # noqa: BLE001
        verdict["coverage_error"] = f"{type(error).__name__}: {str(error)[:300]}"
        print(json.dumps(verdict, ensure_ascii=False))
        return
    for alpha, expected in report["expected"].items():
        same = {
            key: got[alpha][key] == expected[key]
            for key in ("fp_geometry", "fp_meta", "fp_counters", "fp_host_counters",
                        "coverage_outcome")
        }
        verdict[alpha] = {
            "identical": all(same.values()),
            "fields": same,
            "coverage_seconds_in_consumer": got[alpha]["seconds"],
            "coverage_seconds_in_producer": expected["seconds"],
        }
    print(json.dumps(verdict, ensure_ascii=False))
    (Path(directory) / f"patch{patch_id}.consume.json").write_text(
        json.dumps(verdict, ensure_ascii=False, indent=1), encoding="utf-8"
    )


if __name__ == "__main__":
    mode, patch_id, directory = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    {"produce": produce, "consume": consume}[mode](patch_id, directory)
