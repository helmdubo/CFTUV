"""Шаг 0: чем на самом деле является время события волнового фронта.

Стенд отвечает на четыре вопроса числами, а не рассуждением:

1. Рационально ли время edge-события? Если нет — какова степень алгебраического
   числа? Степень меряется `sympy.minimal_polynomial`, а не выводится.
2. То же для split-события.
3. Сколько запросов на сравнение закрывает интервальный фильтр. Меряются ДВА
   фильтра: наш целочисленный (`SqrtSumV1.certified_sign`) и уже имеющийся в
   ядре `planar_types._certified_interval_sign` — карточка называет именно его.
4. Есть ли формулировка, в которой сравнение рационально.

Запуск:
    PYTHONPATH=kernel/src:tools python3 tools/benchmark_wavefront_event_algebra.py
    PYTHONPATH=kernel/src:tools python3 tools/benchmark_wavefront_event_algebra.py --json out.json

Стенд ничего не меняет и ни от чего, кроме ядра и SymPy, не зависит. Скорость
он НЕ меряет — он меряет степень и долю закрытых запросов, и потому кеши не
трогает; временные замеры живут в `tools/benchmark_wavefront_scaling.py`, где
кеши сбрасываются перед каждой точкой.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import json

import sympy as sp

from cftuv_envelope.reference.planar_types import _certified_interval_sign
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.event_time import (
    EventTimeOutcome,
    EventTimeV1,
    SupportLineV1,
    concurrency_time,
)
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.sqrt_sum import SIGN_COUNTS, SqrtSumV1, reset_sign_counts

import wavefront_corpus


def _lines(polygon: PolygonV1) -> list[list[SupportLineV1]]:
    return [
        [
            SupportLineV1.through(
                loop.points[index], loop.points[(index + 1) % len(loop.points)]
            )
            for index in range(len(loop.points))
        ]
        for loop in polygon.loops
    ]


def _sympy_time(time: EventTimeV1) -> sp.Expr:
    divisor = sum(
        sp.Integer(coefficient.numerator)
        / sp.Integer(coefficient.denominator)
        * sp.sqrt(sp.Integer(radicand))
        for radicand, coefficient in time.divisor.terms
    )
    # Без `nsimplify`: на полевых числах он «упрощает» точное выражение в
    # другое и выдаёт степень 175 у величины, которая по построению лежит в
    # поле степени не выше 8. Замер обязан мерить арифметику, а не эвристику.
    return sp.Rational(time.dividend.numerator, time.dividend.denominator) / divisor


def _degree(time: EventTimeV1) -> int:
    return int(sp.degree(sp.minimal_polynomial(_sympy_time(time), sp.Symbol("z"))))


def collect_times(
    polygon: PolygonV1,
) -> tuple[list[EventTimeV1], list[EventTimeV1]]:
    """Времена edge- и split-кандидатов ОДНОГО многоугольника, без цикла."""

    edge_times: list[EventTimeV1] = []
    split_times: list[EventTimeV1] = []
    loops = _lines(polygon)
    flat = [line for loop in loops for line in loop]
    for loop_index, loop in enumerate(polygon.loops):
        lines = loops[loop_index]
        size = len(lines)
        reflex = loop.reflex_flags()
        for index in range(size):
            time, outcome = concurrency_time(
                lines[index],
                lines[(index + 1) % size],
                lines[(index + 2) % size],
            )
            if outcome is EventTimeOutcome.EXACT and time is not None:
                edge_times.append(time)
            # Вершина `index + 1` — стык рёбер `index` и `index + 1`.
            if not reflex[(index + 1) % size]:
                continue
            for far in flat:
                if far in (lines[index], lines[(index + 1) % size]):
                    continue
                time, outcome = concurrency_time(
                    lines[index], lines[(index + 1) % size], far
                )
                if outcome is EventTimeOutcome.EXACT and time is not None:
                    split_times.append(time)
    return edge_times, split_times


def measure_degrees(limit_per_case: int) -> dict:
    """Вопросы 1 и 2: степень алгебраического числа, замеренная SymPy."""

    edge_histogram: dict[int, int] = {}
    split_histogram: dict[int, int] = {}
    radicands: dict[int, int] = {}
    for _, polygon in wavefront_corpus.named_corpus():
        edge_times, split_times = collect_times(polygon)
        for time in edge_times[:limit_per_case]:
            degree = _degree(time)
            edge_histogram[degree] = edge_histogram.get(degree, 0) + 1
            count = len(time.divisor.terms)
            radicands[count] = radicands.get(count, 0) + 1
        for time in split_times[:limit_per_case]:
            degree = _degree(time)
            split_histogram[degree] = split_histogram.get(degree, 0) + 1
    return {
        "edge_degree_histogram": dict(sorted(edge_histogram.items())),
        "split_degree_histogram": dict(sorted(split_histogram.items())),
        "divisor_term_histogram": dict(sorted(radicands.items())),
    }


def measure_filters(limit_per_case: int) -> dict:
    """Вопросы 3 и 4: чем закрывается сравнение двух времён.

    Сравниваются пары СОСЕДНИХ по построению кандидатов одного многоугольника —
    именно такие пары очередь и сравнивает, а случайная пара из разных фигур
    была бы разделена тривиально и завысила бы долю фильтра.
    """

    integer_closed = kernel_closed = kernel_undecided = 0
    rational_only = zero_difference = total = 0
    nonzero = 0
    radicand_histogram: dict[int, int] = {}
    for _, polygon in wavefront_corpus.named_corpus():
        edge_times, split_times = collect_times(polygon)
        times = (edge_times + split_times)[: limit_per_case * 2]
        for left, right in zip(times, times[1:]):
            difference = right.divisor.scaled(left.dividend) - left.divisor.scaled(
                right.dividend
            )
            total += 1
            if difference.is_zero:
                zero_difference += 1
                rational_only += 1
                continue
            nonzero += 1
            count = len(difference.terms)
            radicand_histogram[count] = radicand_histogram.get(count, 0) + 1
            if count == 1 and difference.terms[0][0] == 1:
                rational_only += 1
            if difference.certified_sign(64) is not None:
                integer_closed += 1
            expression = _sympy_difference(difference)
            certified = _certified_interval_sign(expression)
            if certified is None:
                kernel_undecided += 1
            else:
                kernel_closed += 1
    return {
        "comparisons": total,
        "nonzero_comparisons": nonzero,
        "integer_enclosure_closed": integer_closed,
        "kernel_certified_interval_closed": kernel_closed,
        "kernel_certified_interval_undecided": kernel_undecided,
        "rational_only": rational_only,
        "exactly_equal_times": zero_difference,
        "difference_term_histogram": dict(sorted(radicand_histogram.items())),
    }


def _sympy_difference(value: SqrtSumV1) -> sp.Expr:
    return sum(
        sp.Integer(coefficient.numerator)
        / sp.Integer(coefficient.denominator)
        * sp.sqrt(sp.Integer(radicand))
        for radicand, coefficient in value.terms
    )


def measure_queue_signs() -> dict:
    """Что решает знак в ЖИВОМ прогоне очереди, а не в модельной выборке."""

    reset_sign_counts()
    for _, polygon in wavefront_corpus.named_corpus():
        skeleton_module.build_skeleton(polygon)
    return dict(SIGN_COUNTS)


def measure_rational_inputs() -> dict:
    """Вопрос 4, вторая половина: когда время всё-таки рационально."""

    rational_cases = 0
    total_cases = 0
    per_case: dict[str, str] = {}
    for name, polygon in wavefront_corpus.named_corpus():
        edge_times, split_times = collect_times(polygon)
        times = edge_times + split_times
        if not times:
            continue
        total_cases += 1
        rational = all(time.divisor.is_rational() for time in times)
        rational_cases += int(rational)
        per_case[name] = "рационально" if rational else "иррационально"
    return {
        "cases": total_cases,
        "all_times_rational": rational_cases,
        "per_case": per_case,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=str, default=None)
    parser.add_argument("--limit", type=int, default=12)
    arguments = parser.parse_args()

    report = {
        "degrees": measure_degrees(arguments.limit),
        "filters": measure_filters(arguments.limit),
        "queue_signs": measure_queue_signs(),
        "rational_inputs": measure_rational_inputs(),
    }
    _print(report)
    if arguments.json:
        with open(arguments.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)


def _print(report: dict) -> None:
    degrees = report["degrees"]
    print("1-2. СТЕПЕНЬ ВРЕМЕНИ СОБЫТИЯ (sympy.minimal_polynomial)")
    print(f"{'степень':>9} | {'edge':>7} | {'split':>7}")
    all_degrees = sorted(
        {int(key) for key in degrees["edge_degree_histogram"]}
        | {int(key) for key in degrees["split_degree_histogram"]}
    )
    for degree in all_degrees:
        edge = degrees["edge_degree_histogram"].get(degree, 0)
        split = degrees["split_degree_histogram"].get(degree, 0)
        print(f"{degree:>9} | {edge:>7} | {split:>7}")
    print(f"радикалов в знаменателе: {degrees['divisor_term_histogram']}")

    filters = report["filters"]
    print("\n3. ЧЕМ ЗАКРЫВАЕТСЯ СРАВНЕНИЕ ДВУХ ВРЕМЁН")
    total = filters["comparisons"]
    nonzero = filters["nonzero_comparisons"]
    for key, base in (
        ("exactly_equal_times", total),
        ("rational_only", nonzero),
        ("integer_enclosure_closed", nonzero),
        ("kernel_certified_interval_closed", nonzero),
        ("kernel_certified_interval_undecided", nonzero),
    ):
        value = filters[key]
        share = 100.0 * value / base if base else 0.0
        print(f"  {key:<38} {value:>6} из {base:<6} {share:6.2f}%")
    print(
        "  различных радикалов в разности: "
        f"{filters['difference_term_histogram']}"
    )

    print("\n3b. ЖИВОЙ ПРОГОН ОЧЕРЕДИ: чем решён знак")
    signs = report["queue_signs"]
    total_signs = signs["total"]
    for key, value in signs.items():
        if key == "total":
            continue
        share = 100.0 * value / total_signs if total_signs else 0.0
        print(f"  {key:<38} {value:>6} из {total_signs:<6} {share:6.2f}%")

    rational = report["rational_inputs"]
    print("\n4. КОГДА ВРЕМЯ РАЦИОНАЛЬНО")
    print(
        f"  фигур с полностью рациональными временами: "
        f"{rational['all_times_rational']} из {rational['cases']}"
    )
    for name, verdict in rational["per_case"].items():
        print(f"    {name:<34} {verdict}")


if __name__ == "__main__":
    main()
