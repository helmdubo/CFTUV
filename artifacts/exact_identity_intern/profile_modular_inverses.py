"""Профиль одного домена в ОБОИХ представлениях: где остался `pow`.

Ворота карточки стоят на СЧЁТЧИКАХ, а не на секундах (урок ошибки №19), и
главный счётчик назван прямо: модульные обратные на путях тождества.
`Fraction.__hash__` считает ровно один `pow(denominator, -1, 2**61 - 1)` на
вызов, поэтому «сколько модульных обратных» читается из профиля двумя
строками: `builtins.pow` и `fractions.__hash__`.

Печатается ещё и суммарное время профилированного прогона: профиль сам по себе
медленнее боевого, поэтому его секунды сравнимы ТОЛЬКО между собой, и в
расписку карточки они идут как отношение, а не как цена домена.

Прогон из каталога `artifacts/perf_prepare_diag`:
    PYTHONPATH=. python3 ../exact_identity_intern/profile_modular_inverses.py \
        <out.json> [snapshot.json] [density] [alpha]
"""

from __future__ import annotations

import cProfile
import json
import pstats
import sys

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.wavefront.exact_identity import (
    ExactIdentityModeV1,
    set_identity_mode,
)

_WATCHED = (
    ("builtins.pow", ("~", 0, "<built-in function pow>")),
    ("fractions.Fraction.__hash__", None),
)


def _rows(stats: pstats.Stats) -> dict:
    watched = {"builtins.pow": {"calls": 0, "tottime": 0.0}}
    total = 0.0
    ranked = []
    for func, (_, _, tottime, cumtime, _) in stats.stats.items():
        filename, lineno, name = func
        calls = stats.stats[func][0]
        total += tottime
        ranked.append(
            (tottime, f"{filename.rsplit('/', 1)[-1]}:{lineno}:{name}", calls)
        )
        # Имя встроенной функции в профиле CPython пишется двумя способами в
        # зависимости от того, как она вызвана; ловим оба, иначе строка `pow`
        # молча остаётся нулём и расписка утверждает победу, которой не мерила.
        if name in ("<built-in function pow>", "<built-in method builtins.pow>"):
            watched["builtins.pow"] = {
                "calls": calls,
                "tottime": round(tottime, 3),
            }
        if name == "__hash__" and filename.endswith("fractions.py"):
            watched["Fraction.__hash__"] = {
                "calls": calls,
                "cumtime": round(cumtime, 3),
            }
    watched["profiled_total_tottime"] = round(total, 3)
    # Куда уходят ОСТАВШИЕСЯ секунды. Без этой строки расписка называла бы
    # только снятую цену и молчала бы о следующей.
    ranked.sort(reverse=True)
    watched["top_by_tottime"] = [
        {"site": site, "calls": calls, "tottime": round(tottime, 3)}
        for tottime, site, calls in ranked[:8]
    ]
    return watched


def main() -> None:
    out = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "wall_2_001_snapshot.json"
    density = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    alpha = sys.argv[4] if len(sys.argv) > 4 else "0.45"

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
        key = f"{which}:d{density}:a{alpha}:patch{patch_id}"
        results[key] = {}
        for label, mode in (
            ("legacy", ExactIdentityModeV1.LEGACY_TUPLE),
            ("cached", ExactIdentityModeV1.CACHED),
        ):
            previous = set_identity_mode(mode)
            canon.reset_factorization_memory()
            canon.reset_unbudgeted_work()
            profiler = cProfile.Profile()
            profiler.enable()
            try:
                prepared, domain = run_queue_domain(
                    patch_id, domain_id, snapshot, request, alpha
                )
            finally:
                profiler.disable()
                set_identity_mode(previous)
            row = _rows(pstats.Stats(profiler))
            row["outcome"] = (
                f"{domain.preparation_outcome}/{domain.coverage_outcome}"
            )
            budget = getattr(prepared, "work_budget", None)
            row["units"] = (
                dict(budget.counters())["EXACT_WORK_SPENT"] if budget else None
            )
            results[key][label] = row
        print(key, json.dumps(results[key], ensure_ascii=False), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
