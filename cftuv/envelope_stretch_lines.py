"""Строки консоли: сколько растяжения израсходовала каждая развёртка из бюджета.

Бюджет растяжения развёртки (`DEVELOPABLE_STRETCH_BUDGET` ядра) — решение владельца, а
принятая карта может лежать где угодно внутри него: 0.1 % или 19.9 %. Артист должен видеть,
сколько ушло на самом деле, поэтому каждый домен-развёртка получает строку

    [CFTUV][Production] STRETCH patch 3 (domain ...a1b2c3): stretch <= 14.2 % (budget 20 %)

Числа берутся из диагностики ядра `DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES`
(`worst_band_squared<=B stretch_budget=b`): `B` — сертифицированная ВЕРХНЯЯ граница квадрата
сингулярного числа отображения источник -> карта, значит наибольшее отношение длин — `sqrt(B)`,
а растяжение — `sqrt(B) - 1`. Процент округляется ВВЕРХ до 0.1: строка «не больше» остаётся
верной. Хост ничего не решает и не пересчитывает: судит ядро, а здесь только показ.
"""

from __future__ import annotations

import math
import re

#: Имя диагностики ядра, несущей числа развёртки (`NamedOutcome`), и разбор её чисел.
DEVELOPABLE_DIAGNOSTIC = "DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES"
_NUMBERS = re.compile(r"worst_band_squared<=([0-9.eE+-]+) stretch_budget=([0-9.eE+-]+)")


def _percent_up(value: float) -> float:
    """Процент с округлением вверх до десятой доли (строка «не больше» не врёт)."""

    return math.ceil(value * 10.0 - 1e-9) / 10.0


def developable_stretches(results) -> list[tuple]:
    """`(patch_id, domain_id, растяжение %, бюджет %)` каждого домена-развёртки, по номеру патча."""

    found = []
    for item in results:
        for line in item.diagnostics:
            if not line.startswith(DEVELOPABLE_DIAGNOSTIC):
                continue
            numbers = _NUMBERS.search(line)
            if numbers is None:
                continue
            band, budget = float(numbers.group(1)), float(numbers.group(2))
            stretch = max(0.0, math.sqrt(max(band, 1.0)) - 1.0) * 100.0
            found.append(
                (item.patch_id, item.domain_id, _percent_up(stretch), budget * 100.0)
            )
    return sorted(found, key=lambda row: (row[0], str(row[1])))


def developable_stretch_lines(results) -> list[str]:
    """Строка на каждый домен-развёртку; при нескольких — ещё итог с наибольшим растяжением."""

    rows = developable_stretches(results)
    lines = [
        f"[CFTUV][Production] STRETCH patch {patch} (domain ...{str(domain)[-6:]}): "
        f"stretch <= {stretch:.1f} % (budget {budget:g} %)"
        for patch, domain, stretch, budget in rows
    ]
    if len(rows) > 1:
        patch, _domain, stretch, budget = max(rows, key=lambda row: row[2])
        lines.append(
            f"[CFTUV][Production] STRETCH: {len(rows)} developable domains, the largest "
            f"stretch <= {stretch:.1f} % (patch {patch}, budget {budget:g} %)"
        )
    return lines
