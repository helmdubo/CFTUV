"""Полный обсчёт ВСЕХ доменов здания под штатным капом — знаменатель релиза.

Требование четвёртого внешнего аудита: полевая матрица проверяет
представительные патчи, и этого достаточно для быстрых ворот на каждое
изменение, но утверждение «всё здание больше не висит и имеет
утверждённые исходы» обязано опираться на прогон всех доменов сцены.
Этот скрипт и есть тот прогон: каждый домен считается тем же маршрутом,
что и в Blender (подготовка → конвейер → скелет → грани → покрытие),
под штатным потолком работы; исход, шесть статей расхода, секунды и
деталь отказа записываются в расписку.

Прогон: `PYTHONPATH=. python3 ../building_full_sweep/sweep.py <куда.json>`
из каталога `artifacts/perf_prepare_diag` (там живёт `env` с путями).
"""

from __future__ import annotations

import json
import sys
import time

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon


def main():
    out = sys.argv[1]
    import big_scene
    from cftuv.envelope_queue_export import run_queue_domain
    from cftuv.envelope_request_export import (
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected
    )

    from cftuv.envelope_request_export import EnvelopeHostAdapterError

    results = {}
    for patch_id in sorted(patch_ids):
        domain_id = _typed_value("patch-domain", revision, patch_id)
        canon.reset_factorization_memory()
        canon.reset_unbudgeted_work()
        started = time.perf_counter()
        # Именованный отказ хоста на входе (например, домен не плоский в
        # пределах объявленного near-planar бюджета) — это ИСХОД домена в
        # знаменателе, а не авария обсчёта: записываем и идём дальше.
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle, included_patch_ids=frozenset({patch_id})
            )
            request = build_envelope_decal_request(
                snapshot,
                frozenset(by_domain[domain_id]),
                0.45,
                decal_request_id_value=request_id,
                density=0,
            )
            prepared, domain = run_queue_domain(
                patch_id, domain_id, snapshot, request, "0.45"
            )
        except EnvelopeHostAdapterError as refusal:
            row = {
                "seconds": round(time.perf_counter() - started, 3),
                "outcome": "HOST_ADMISSION_REFUSED",
                "leaked_unbudgeted": canon.UNBUDGETED_WORK.spent,
                "detail": str(refusal),
            }
            results[f"patch{patch_id}"] = row
            print(
                f"patch{patch_id}",
                json.dumps(row, ensure_ascii=False),
                flush=True,
            )
            continue
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
        results[f"patch{patch_id}"] = row
        print(f"patch{patch_id}", json.dumps(row, ensure_ascii=False), flush=True)

    outcomes = {}
    for row in results.values():
        outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
    spent = [row.get("EXACT_WORK_SPENT", 0) for row in results.values()]
    summary = {
        "domains": len(results),
        "outcomes": outcomes,
        "max_units": max(spent) if spent else 0,
        "total_seconds": round(
            sum(row["seconds"] for row in results.values()), 3
        ),
        "max_seconds": max(
            (row["seconds"] for row in results.values()), default=0
        ),
        "leaked_unbudgeted_total": sum(
            row["leaked_unbudgeted"] for row in results.values()
        ),
    }
    print("SUMMARY", json.dumps(summary, ensure_ascii=False), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(
            {"summary": summary, "domains": results},
            handle, ensure_ascii=False, indent=2, sort_keys=True,
        )


if __name__ == "__main__":
    main()
