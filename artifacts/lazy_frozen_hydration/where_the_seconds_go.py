"""КУДА уходят секунды 2.001, если единицы уже вернулись к эталону.

Вопрос поставлен числом, а не догадкой. Ленивая гидратация вернула расход
домена к мишени (24 030 против 24 508), а секунды не сдвинулись. Значит
секунды тратит НЕ гидратация, и назвать, что именно, обязан замер.

Здесь мерятся ТРИ прогона одного домена в ОДНОМ процессе, подряд, чтобы разница
не оказалась разницей загрузки машины:

1. `lazy` — рабочий путь;
2. `lazy+cached_fraction_hash` — КОНТРФАКТ: `Fraction.__hash__` подменён
   кэширующим. Значение хэша при этом ТО ЖЕ (кэш, а не другой хэш), поэтому
   ответ обязан совпасть до байта, а разница секунд называет цену тождества
   точных величин: ключи вхождений и ключи портов — кортежи с `Fraction`
   внутри, и каждый словарный вопрос про них считает модульную инверсию;
3. `lazy` ещё раз — контроль дрейфа машины: если третий прогон разошёлся с
   первым сильнее, чем контрфакт, измерение мерило загрузку, а не код.

Подмена живёт ВНУТРИ процесса замера. В ядро она не идёт: кэш по `id(self)`
корректен только пока объект жив, и построить на нём поставку нельзя.

Прогон из `artifacts/perf_prepare_diag`:
    PYTHONPATH=. python3 ../lazy_frozen_hydration/where_the_seconds_go.py <out.json>
"""

from __future__ import annotations

import fractions
import json
import sys
import time

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon

_ORIGINAL_HASH = fractions.Fraction.__hash__
_CACHE: dict[int, int] = {}
_ALIVE: list = []


def _cached_hash(self):
    key = id(self)
    value = _CACHE.get(key)
    if value is None:
        value = _ORIGINAL_HASH(self)
        _CACHE[key] = value
        # Ссылку держим, иначе `id` переиспользуется и кэш начнёт врать.
        _ALIVE.append(self)
    return value


def _provider(dense: bool):
    from cftuv_envelope.wavefront import prepare_conveyor

    def provide(patch_id, domain_id, selected, snapshot, request):
        return prepare_conveyor(snapshot, request, dense_hydration=dense)

    return provide


def _answer(prepared, domain):
    from cftuv_envelope.wavefront.digest import semantic_digest

    return (
        str(domain.preparation_outcome),
        str(domain.coverage_outcome),
        tuple(
            (
                repr(region.skeleton_outcome),
                repr(region.face_outcome),
                None if region.skeleton is None
                else semantic_digest(region.skeleton),
                None if region.partition is None
                else repr(region.partition.doubled_area.terms),
            )
            for region in prepared.regions
        ),
    )


def main():
    out = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "wall_2_001_snapshot.json"
    density = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot
    from cftuv.envelope_queue_export import run_queue_domain

    path = env.SNAPSHOTS / which
    payload = load_snapshot(path)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(path)
    rows = snapshot_and_request(bundle, selected, density=density)
    results = {}
    for patch_id, domain_id, snapshot, request in rows:
        answers = {}
        row = {}
        for label, cached in (
            ("lazy", False),
            ("lazy_cached_fraction_hash", True),
            ("lazy_control", False),
        ):
            fractions.Fraction.__hash__ = (
                _cached_hash if cached else _ORIGINAL_HASH
            )
            canon.reset_factorization_memory()
            canon.reset_unbudgeted_work()
            started = time.perf_counter()
            prepared, domain = run_queue_domain(
                patch_id, domain_id, snapshot, request, "0.45",
                preparation_provider=_provider(False),
            )
            seconds = time.perf_counter() - started
            fractions.Fraction.__hash__ = _ORIGINAL_HASH
            budget = getattr(prepared, "work_budget", None)
            row[label] = {
                "seconds": round(seconds, 3),
                "units": (
                    dict(budget.counters())["EXACT_WORK_SPENT"]
                    if budget else None
                ),
            }
            answers[label] = _answer(prepared, domain)
        assert len(set(answers.values())) == 1, ("ANSWERS_DIVERGED", patch_id)
        row["answers_agree"] = True
        results[f"{which}:d{density}:patch{patch_id}"] = row
        print(json.dumps(results, ensure_ascii=False, indent=1), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
