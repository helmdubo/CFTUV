"""КУДА уходят единицы: до марша фронта или во время него.

Вопрос не праздный, он определяет ОЖИДАНИЕ от будущего перемера. Если расход
почти весь платится ДО обхода — на канонизации радикандов, вытекающих из
скоростей полигона, — то починка walls.012 (обход дойдёт до 12 узлов вместо 2)
НЕ должна заметно сдвинуть число. Если же расход растёт вместе с длиной обхода,
починка сделает здоровый домен дороже, и запас потолка придётся пересчитывать.
Гадать тут нечего: у бюджета уже есть поле, которое отвечает.

РАЗДЕЛИТЕЛЬ. `skeleton._Builder` пишет в `budget.superlevel` номер уровня перед
каждым `_apply_level`, и до первого уровня поле пустое. Значит списание с
пустым `superlevel` сделано ДО марша (простой базис полигона, граф мотоциклов,
подготовка), а с непустым — ВНУТРИ марша. Поле уже есть в контракте, поэтому
разделитель ничего не изобретает: он читает то, что отказ и так печатает.

Прогон: `PYTHONPATH=. python3 ../exact_work_budget/where_the_units_go.py <out.json>`
из каталога `artifacts/perf_prepare_diag`.
"""

from __future__ import annotations

import json
import sys

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon

_ARTICLES = (
    "modular_squarings",
    "gcd_operations",
    "miller_rabin_rounds",
    "pollard_attempts",
    "radical_materializations",
    "exact_position_hydrations",
)


def _install(tally):
    """Обернуть все шесть списаний ОДИН РАЗ, разложив их по фазе и операции.

    Один раз — не стилистика. Повторная установка оборачивает уже обёрнутое, и
    старая обёртка продолжает писать в СВОЙ словарь на всех последующих
    доменах: у первого домена число вырастает за счёт чужой работы. Так и
    случилось на первом прогоне (walls.012 d0 показал 621 285 вместо 554 662),
    и поймано это было сверкой с расписками, а не чтением кода.
    """

    budget_class = canon.ExactWorkBudgetV1
    for article in _ARTICLES:
        name = f"spend_{article}"
        original = getattr(budget_class, name)

        def make(original=original, article=article):
            def spend(self, count, *, operation, radicand):
                phase = (
                    "BEFORE_THE_MARCH" if not self.superlevel
                    else "DURING_THE_MARCH"
                )
                key = f"{self.stage}/{phase}"
                row = tally.setdefault(key, {"units": 0, "by_operation": {}})
                row["units"] += count
                row["by_operation"][operation.value] = (
                    row["by_operation"].get(operation.value, 0) + count
                )
                row.setdefault("by_article", {})
                row["by_article"][article] = (
                    row["by_article"].get(article, 0) + count
                )
                return original(
                    self, count, operation=operation, radicand=radicand
                )

            return spend

        setattr(budget_class, name, make())


def _rows():
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

    results = {}
    tally: dict[str, dict] = {}
    _install(tally)
    for label, (patch_id, domain_id, snapshot, request) in _rows():
        tally.clear()
        canon.reset_factorization_memory()
        canon.reset_unbudgeted_work()
        _, domain = run_queue_domain(
            patch_id, domain_id, snapshot, request, "0.45"
        )
        before = sum(
            row["units"] for key, row in tally.items()
            if key.endswith("BEFORE_THE_MARCH")
        )
        during = sum(
            row["units"] for key, row in tally.items()
            if key.endswith("DURING_THE_MARCH")
        )
        total = before + during
        results[f"{label}:patch{patch_id}"] = {
            "outcome": (
                f"{domain.preparation_outcome}/{domain.coverage_outcome}"
            ),
            "units_before_the_march": before,
            "units_during_the_march": during,
            "share_before_the_march": (
                round(before / total, 5) if total else None
            ),
            # Копия, а не ссылка: словарь один на все домены и очищается
            # перед каждым, иначе расписка хранила бы последний домен трижды.
            "phases": json.loads(json.dumps(tally)),
        }
        print(
            f"{label}:patch{patch_id}  до марша {before}  в марше {during}  "
            f"доля до марша {before / total:.4%}" if total else label,
            flush=True,
        )
    json.dump(results, open(out, "w", encoding="utf-8"),
              ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
