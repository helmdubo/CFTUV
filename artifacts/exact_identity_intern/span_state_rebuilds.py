"""Сколько РАЗ пересобирается состояние пролёта — в обоих представлениях.

Вторая половина ворот карточки стоит на этом счётчике: «число пересборок
`span_state` ограничено числом различных (t, вхождение, поколение)». Время и
поколение у вида одни, значит остаётся вхождение, то есть ЛИСТ, и граница
читается как «не больше, чем различных листов, суммарно по всем видам».

Считается конструкция `CandidateSpanStateV1` — единственное место, где пролёт
собирается заново. Подмена живёт внутри замера; ответ домена от неё не
двигается, и это проверяется здесь же сверкой исходов и единиц.

Прогон из каталога `artifacts/perf_prepare_diag`:
    PYTHONPATH=. python3 ../exact_identity_intern/span_state_rebuilds.py \
        <out.json> [snapshot.json] [density] [alpha]
"""

from __future__ import annotations

import json
import sys

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.wavefront import symbolic_overlay as overlay_module
from cftuv_envelope.wavefront.exact_identity import (
    ExactIdentityModeV1,
    set_identity_mode,
)


def main() -> None:
    out = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "wall_2_001_snapshot.json"
    density = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    alpha = sys.argv[4] if len(sys.argv) > 4 else "0.45"

    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot
    from cftuv.envelope_queue_export import run_queue_domain

    original = overlay_module.CandidateSpanStateV1
    built = [0]
    distinct = [set()]

    def counting(line, span, start, end, *rest):
        built[0] += 1
        return original(line, span, start, end, *rest)

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
            built[0] = 0
            distinct[0] = set()
            canon.reset_factorization_memory()
            canon.reset_unbudgeted_work()
            overlay_module.CandidateSpanStateV1 = counting
            try:
                prepared, domain = run_queue_domain(
                    patch_id, domain_id, snapshot, request, alpha
                )
            finally:
                overlay_module.CandidateSpanStateV1 = original
                set_identity_mode(previous)
            budget = getattr(prepared, "work_budget", None)
            results[key][label] = {
                "span_state_rebuilds": built[0],
                "outcome": (
                    f"{domain.preparation_outcome}/{domain.coverage_outcome}"
                ),
                "units": (
                    dict(budget.counters())["EXACT_WORK_SPENT"]
                    if budget else None
                ),
            }
        assert (
            results[key]["legacy"]["outcome"] == results[key]["cached"]["outcome"]
            and results[key]["legacy"]["units"] == results[key]["cached"]["units"]
        ), key
        print(key, json.dumps(results[key], ensure_ascii=False), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
