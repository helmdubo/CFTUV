"""ГДЕ именно точная работа зовётся мимо бюджета — поимённо и с числами.

Открытый счёт Q-FACES-SIGN-OUTSIDE-THE-BUDGET назывался «faces.py зовёт
`sign()` мимо бюджета». Это ОКАЗАЛОСЬ УЖЕ, чем правда: непротянутых мест
больше, и они живут не только в `faces.py`. Врать в сторону «у нас всего одна
дырка» карточка не имеет права, поэтому счёт здесь ПЕРЕСЧИТАН, а не пересказан.

Что меряется. `SqrtSumV1.sign` оборачивается счётчиком, который для каждого
вызова записывает МЕСТО ВЫЗОВА и то, был ли назван бюджет; отдельно берётся
прирост телеметрии `UNBUDGETED_WORK` ровно вокруг вызова. Так один прогон
отвечает на оба вопроса сразу: сколько мест зовут `sign()` без бюджета и
сколько ЕДИНИЦ они за собой оставили. Второе число — то, ради чего вопрос
вообще задан: место без бюджета опасно тогда и только тогда, когда оно платит.

Прогон: `PYTHONPATH=. python3 ../exact_work_budget/unbudgeted_sites.py <out.json>`
из каталога `artifacts/perf_prepare_diag`.
"""

from __future__ import annotations

import json
import sys
import sysconfig

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon

_ROOT = str(env.WT)


def _install_probe(sites):
    original = canon.SqrtSumV1.sign

    def _where(depth):
        # +1: сам `_where` — тоже кадр. Забыть про это значило бы записать
        # обёртку вместо места вызова, что уже случилось однажды.
        frame = sys._getframe(depth + 1)
        text = f"{frame.f_code.co_filename}:{frame.f_lineno}"
        return text[len(_ROOT) + 1:] if text.startswith(_ROOT) else text

    def probed(self, *, filter_bits: int = 64, budget=None):
        before = canon.UNBUDGETED_WORK.spent
        result = original(self, filter_bits=filter_bits, budget=budget)
        row = sites.setdefault(
            _where(1),
            {
                "calls": 0,
                "with_budget": 0,
                "without_budget": 0,
                "units_charged_without_budget": 0,
                # Место, откуда пришёл БЕЗБЮДЖЕТНЫЙ вызов. Для `compare_times`
                # и подобных обёрток сама строка `sign()` ничего не говорит:
                # бюджет теряется у ВЫЗЫВАЮЩЕГО, и чинить придётся там.
                "unbudgeted_callers": {},
            },
        )
        row["calls"] += 1
        if budget is not None:
            row["with_budget"] += 1
        else:
            row["without_budget"] += 1
            row["units_charged_without_budget"] += (
                canon.UNBUDGETED_WORK.spent - before
            )
            caller = _where(2)
            row["unbudgeted_callers"][caller] = (
                row["unbudgeted_callers"].get(caller, 0) + 1
            )
        return result

    canon.SqrtSumV1.sign = probed
    return original


def _domains():
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot

    for name, density in (
        ("walls_012_snapshot.json", 0),
        ("walls_012_snapshot.json", 1),
        ("wall_2_001_snapshot.json", 0),
    ):
        path = env.SNAPSHOTS / name
        payload = load_snapshot(path)
        selected = frozenset(payload["raw"]["selected_edges"])
        _, _, bundle = bundle_from_field_snapshot(path)
        for row in snapshot_and_request(bundle, selected, density=density):
            yield f"{name}:d{density}", row


def main() -> None:
    out = sys.argv[1]
    from cftuv.envelope_queue_export import run_queue_domain

    sites: dict[str, dict[str, int]] = {}
    _install_probe(sites)
    outcomes = {}
    for label, (patch_id, domain_id, snapshot, request) in _domains():
        canon.reset_factorization_memory()
        canon.reset_unbudgeted_work()
        _, domain = run_queue_domain(
            patch_id, domain_id, snapshot, request, "0.45"
        )
        outcomes[f"{label}:patch{patch_id}"] = (
            f"{domain.preparation_outcome}/{domain.coverage_outcome}"
        )
    report = {
        "outcomes": outcomes,
        "sites": dict(sorted(sites.items())),
        "sites_with_unbudgeted_calls": sorted(
            where for where, row in sites.items() if row["without_budget"]
        ),
        "unbudgeted_calls": sum(row["without_budget"] for row in sites.values()),
        "units_charged_without_budget": sum(
            row["units_charged_without_budget"] for row in sites.values()
        ),
    }
    json.dump(report, open(out, "w", encoding="utf-8"),
              ensure_ascii=False, indent=2, sort_keys=True)
    print(json.dumps(
        {
            "sites_with_unbudgeted_calls": len(
                report["sites_with_unbudgeted_calls"]
            ),
            "unbudgeted_calls": report["unbudgeted_calls"],
            "units_charged_without_budget": report[
                "units_charged_without_budget"
            ],
        },
        ensure_ascii=False,
    ))


if __name__ == "__main__":
    main()
