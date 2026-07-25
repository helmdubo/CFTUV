"""Измерить, как растёт цена arrangement от числа сегментов границы домена.

Зачем: полевой прогон `building.002` показал, что один патч съедает 11.8 с из
12.4 с, а мелкие патчи того же запроса — по 130 мс. Отношение 268× на
`RAW_UNION` при границе больше в 4–5 раз даёт показатель степени около
3.5–4. Это оценка по двум точкам; стенд даёт кривую.

Замеряется только геометрия: вклад фиксирован, растёт исключительно число
сегментов границы домена. Так виден именно показатель степени, а не разница
входных данных.

Запуск:
    PYTHONPATH=kernel/src python3 tools/benchmark_arrangement_scaling.py
    PYTHONPATH=kernel/src python3 tools/benchmark_arrangement_scaling.py --json out.json

Стенд ничего не меняет и ни от чего не зависит, кроме ядра.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from fractions import Fraction

from cftuv_envelope.reference.common import make_region, make_segment
from cftuv_envelope.reference.arrangement import ExactSegmentArrangementBackend
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
)
from cftuv_envelope.reference.provenance import ReferenceProvenanceV1


def _certificate(name: str) -> ConstructionCertificate:
    return ConstructionCertificate(
        kind=ConstructionKind.SOURCE_VERTEX,
        support_ids=frozenset({name}),
    )


def _region(name: str, points: tuple[tuple[Fraction, Fraction], ...]):
    """Замкнутый контур из точек. Дубли точек запрещены: нулевой сегмент даёт
    неразрешимый предикат, и это будет отказ стенда, а не результат."""

    assert len(set(points)) == len(points), f"{name}: повторяющиеся точки"
    provenance = ReferenceProvenanceV1()
    segments = []
    for index in range(len(points)):
        start = ExactPlanarPoint.from_values(*points[index])
        end = ExactPlanarPoint.from_values(*points[(index + 1) % len(points)])
        segments.append(
            make_segment(
                f"{name}-e{index}",
                start,
                end,
                support_ids=frozenset({f"{name}-support{index}"}),
                provenance=provenance,
                start_certificates=frozenset({_certificate(f"{name}-v{index}")}),
                end_certificates=frozenset(
                    {_certificate(f"{name}-v{(index + 1) % len(points)}")}
                ),
                boundary_constraint_ids=frozenset({f"{name}-constraint{index}"}),
            )
        )
    return make_region(
        f"{name}-region", tuple(segments), instance_id=name, spec_id=name
    )


def _comb_domain(teeth: int):
    """Прямоугольник с пилой на верхней стороне.

    Площадь и положение почти не меняются, растёт только число сегментов
    границы — именно та величина, от которой, по гипотезе, идёт степень.
    """

    points = [(Fraction(0), Fraction(0)), (Fraction(20), Fraction(0))]
    step = Fraction(20, 2 * teeth + 1)
    for index in range(2 * teeth + 1):
        height = Fraction(10) if index % 2 == 0 else Fraction(41, 4)
        points.append((Fraction(20) - step * index, height))
    points.append((Fraction(0), Fraction(10)))
    return _region("domain", tuple(points))


def _contributions(count: int):
    """`count` перекрывающихся полос — модель нескольких источников на патче.

    Это вторая ось, и в поле растёт именно она: одно выделенное ребро даёт
    один вклад, 2000 рёбер — 2000 вкладов. Полосы намеренно пересекаются
    друг с другом, как пересекаются огибающие соседних цепочек.
    """

    regions = []
    for index in range(count):
        low = Fraction(1) + Fraction(index, 2)
        high = Fraction(10) - Fraction(index, 3)
        left = Fraction(1) + Fraction(index, 4)
        right = Fraction(19) - Fraction(index, 5)
        regions.append(
            _region(
                f"contribution{index}",
                ((left, low), (right, low), (right, high), (left, high)),
            )
        )
    return tuple(regions)


def _contribution():
    return (
        _region(
            "contribution",
            # Вклад намеренно перекрывает всю пилу: иначе AABB-фильтр отбросит
            # все пары, пересечений не будет и стенд покажет линейный рост,
            # которого в поле нет.
            (
                (Fraction(1), Fraction(1)),
                (Fraction(19), Fraction(1)),
                (Fraction(19), Fraction(41, 4) + Fraction(1, 2)),
                (Fraction(1), Fraction(41, 4) + Fraction(1, 2)),
            ),
        ),
    )


def measure(
    teeth_counts: tuple[int, ...], contribution_count: int = 1
) -> list[dict]:
    backend = ExactSegmentArrangementBackend()
    contribution = (
        _contribution() if contribution_count == 1
        else _contributions(contribution_count)
    )
    rows: list[dict] = []
    for teeth in teeth_counts:
        domain = (_comb_domain(teeth),)
        segments = len(domain[0].outer.segments) + sum(
            len(item.outer.segments) for item in contribution
        )
        started = time.perf_counter()
        build = backend.build_arrangement(contribution, domain)
        build_seconds = time.perf_counter() - started

        started = time.perf_counter()
        backend.exact_union(contribution, domain, {})
        union_seconds = time.perf_counter() - started

        rows.append(
            {
                "segments": segments,
                "build_seconds": build_seconds,
                "union_seconds": union_seconds,
                "broadphase_candidate_pairs": build.counters.broadphase_candidate_pairs,
                "narrowphase_tests": build.counters.narrowphase_tests,
                "actual_intersections": build.counters.actual_intersections,
                "atomic_edges": build.counters.atomic_edges,
            }
        )
    return rows


def _exponent(rows: list[dict], key: str) -> float | None:
    """Наклон log-log между первой и последней точкой: грубый показатель степени."""

    usable = [row for row in rows if row[key] > 0]
    if len(usable) < 2:
        return None
    first, last = usable[0], usable[-1]
    if first["segments"] == last["segments"]:
        return None
    return math.log(last[key] / first[key]) / math.log(
        last["segments"] / first["segments"]
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=str, default=None)
    parser.add_argument("--max-teeth", type=int, default=10)
    parser.add_argument(
        "--contributions",
        type=int,
        default=1,
        help="число перекрывающихся вкладов; в поле это число выделенных рёбер",
    )
    arguments = parser.parse_args()

    counts = tuple(
        value for value in (1, 2, 3, 4, 6, 8, 10, 14, 20) if value <= arguments.max_teeth
    )
    rows = measure(counts, arguments.contributions)

    print(
        f"=== цена arrangement, вкладов: {arguments.contributions} ===")
    print(
        f"  {'сегм.':>6} {'build, с':>10} {'union, с':>10} "
        f"{'пар':>10} {'пересеч.':>9} {'атомарных':>10}"
    )
    for row in rows:
        print(
            f"  {row['segments']:6d} {row['build_seconds']:10.3f} "
            f"{row['union_seconds']:10.3f} "
            f"{row['broadphase_candidate_pairs']:10d} "
            f"{row['actual_intersections']:9d} {row['atomic_edges']:10d}"
        )

    print("\n=== показатель степени (наклон log-log) ===")
    for key, label in (
        ("build_seconds", "build_arrangement"),
        ("union_seconds", "exact_union"),
        ("broadphase_candidate_pairs", "пар broadphase"),
        ("atomic_edges", "атомарных рёбер"),
    ):
        value = _exponent(rows, key)
        print(f"  {label:20s} O(n^{value:.2f})" if value else f"  {label:20s} —")

    print(
        "\n  Пары broadphase обязаны расти как n^2, пока перебор всех пар не\n"
        "  заменён sweep-line. Число реальных пересечений растёт линейно —\n"
        "  разница между этими двумя строками и есть выброшенная работа."
    )

    if arguments.json:
        with open(arguments.json, "w", encoding="utf-8") as handle:
            json.dump(
                {
                    "schema": "cftuv.envelope.arrangement_scaling.v1",
                    "rows": rows,
                    "exponents": {
                        key: _exponent(rows, key)
                        for key in (
                            "build_seconds",
                            "union_seconds",
                            "broadphase_candidate_pairs",
                            "atomic_edges",
                        )
                    },
                },
                handle,
                indent=2,
            )
        print(f"\n  записано: {arguments.json}")


if __name__ == "__main__":
    main()
