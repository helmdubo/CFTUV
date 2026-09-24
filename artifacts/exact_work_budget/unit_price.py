"""ЦЕНА одной единицы бюджета в секундах — верхняя граница выбора потолка.

Кап объявлен в ЕДИНИЦАХ, а не в секундах, и это не спор: секунда есть свойство
машины, а не входа. Но у выбранного литерала есть вторая, тоже измеренная
граница — ЦЕНА полного потолка. Если она вырастет до минут, именованный отказ
перестанет отличаться от зависания по ощущению владельца: кап потеряет смысл,
не потеряв ни одной буквы.

ЦЕНА МЕРЯЕТСЯ ЧЕРЕЗ БОЕВОЙ ПУТЬ, а не микробенчмарком голого цикла. Голый
`y = (y*y + c) % n` на этой машине втрое дешевле единицы, которую платит
`_rho_factors`: пакетный gcd, произведение разностей и раунды Миллера—Рабина
входят в ту же единицу и в ту же секунду. Считать по голому циклу значило бы
объявить потолок дешевле, чем он есть, — то есть ошибиться в безопасную для
себя сторону.

Ширины взяты не с потолка: 68 бит — типичный радиканд building, 128 — wall
2.001, 246 — МАКСИМУМ, наблюдавшийся на walls.012 (REPORT.txt §3 перф-карточки).
Делитель во всех трёх один и тот же 30-битный простой: цена единицы обязана
зависеть от ШИРИНЫ МОДУЛЯ, и подмена делителя вместе с шириной смешала бы два
эффекта.

Прогон: `python3 artifacts/exact_work_budget/unit_price.py`. Число
машинно-зависимо, поэтому живёт в расписке рядом с именем машины, а НЕ в тесте:
тест на секунды был бы красным на чужом железе.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "kernel" / "src"))

from cftuv_envelope.exact_sqrt_sum import (  # noqa: E402
    _EXACT_CANONICALIZATION_WORK_CAP,
    _rho_factors,
    unlimited_reference_budget,
)

# Один и тот же делитель на всех ширинах: ~2^30, то есть ро-Поллард обязан
# сделать порядка 1.18*sqrt(2^30) ≈ 40 000 шагов орбиты. Этого хватает, чтобы
# цена шага перестала зависеть от разогрева, и мало, чтобы прогон был секундным.
_SMALL_PRIME = 1073741827  # 2^30 + 3, простое

# Кофакторы: простые, добивающие произведение до нужной ширины. Значения
# фиксированы, а не подобраны на прогоне: цена сравнивается с ценой того же
# числа при каждом повторе.
_COFACTORS = {
    68: 137438953481,
    128: 158456325028528675187087900777,
    246: 52656145834278593348959013841835216159447547700274555627155489019,
}


def _widths() -> dict[int, int]:
    return {bits: _SMALL_PRIME * cofactor for bits, cofactor in _COFACTORS.items()}


def _price_ns(n: int) -> tuple[float, int, int]:
    """Наносекунды на ЕДИНИЦУ бюджета при разложении `n` боевым путём."""

    budget = unlimited_reference_budget(stage="UNIT_PRICE")
    started = time.perf_counter()
    factors = _rho_factors(n, budget)
    elapsed = time.perf_counter() - started
    assert factors, "разложение обязано вернуть множители"
    return elapsed * 1e9 / budget.spent, budget.spent, n.bit_length()


def main() -> None:
    cap = _EXACT_CANONICALIZATION_WORK_CAP
    rows: dict[str, object] = {"cap": cap}
    widest_bits = 0
    widest_price = 0.0
    for _, n in sorted(_widths().items()):
        price_ns, spent, bits = _price_ns(n)
        rows[f"{bits}_bit_radicand"] = {
            "ns_per_unit": round(price_ns, 1),
            "units_spent": spent,
        }
        if bits > widest_bits:
            widest_bits, widest_price = bits, price_ns
    rows["widest_bits"] = widest_bits
    rows["cap_price_seconds_at_widest"] = round(widest_price * cap / 1e9, 2)
    json.dump(rows, sys.stdout, ensure_ascii=False, indent=2, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
