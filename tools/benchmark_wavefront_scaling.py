"""Масштабирование очереди событий: время построения от числа вершин.

Против чего меряем. `mesh_inset` Говарда Трики на той же гребёнке с reflex-
зубьями (замер записан в `DECISIONS.md`, 2026-07-25):

    42 -> 0.0084 c, 128 -> 0.1288 c, 256 -> 0.5455 c,
    512 -> 2.6185 c, 1024 -> 100.89 c;  показатель 42->512 = O(n^2.30)

Его код — binary64 с допусками и поиском edge-событий перебором всех спиц;
наш — точная целочисленная арифметика. То есть сравниваются РАЗНЫЕ задачи, и
стенд обязан показывать не «кто быстрее», а ПОКАЗАТЕЛЬ СТЕПЕНИ.

Главная величина стенда — НЕ секунды, а `split_candidates_examined`. Секунды —
следствие; если кандидаты падают, а время нет, значит цена лежала не там, и
стенд обязан это показать, а не спрятать.

База среза Q1 (наивный перебор), перепроверена независимо:

    16 -> 84 кандидата, 128 -> 7812, 894 -> 396940, 2002 -> 1998000
    2002 вершины: 25.78 с, sign_queries 79499, sign_by_conjugation 0

Оба пути поиска мерятся рядом (`--search both`), потому что выигрыш имеет смысл
только против эталона, посчитанного на той же машине в том же прогоне.

Каждая точка меряется на СВЕЖИХ значениях: многоугольник строится заново, а
кеши `squarefree_split`/`prime_support` сбрасываются. Без сброса второй прогон
мерил бы факторизацию из кеша, а не арифметику.

Запуск:
    PYTHONPATH=kernel/src:tools python3 tools/benchmark_wavefront_scaling.py
    PYTHONPATH=kernel/src:tools python3 tools/benchmark_wavefront_scaling.py \
        --teeth 6,62,445,999 --search both
"""

from __future__ import annotations

import argparse
import json
import math
import time

from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton
from cftuv_envelope.wavefront.sqrt_sum import (
    SIGN_COUNTS,
    reset_factorization_memory,
    reset_sign_counts,
)

import wavefront_corpus


# Замеры Трики на той же форме, из `DECISIONS.md`. Не для соревнования, а для
# того, чтобы показатель степени было с чем сопоставить.
TRIKI_SECONDS = {42: 0.0084, 128: 0.1288, 256: 0.5455, 512: 2.6185, 1024: 100.89}


def _fresh() -> None:
    reset_factorization_memory()
    reset_sign_counts()


REPORTED_COUNTERS = (
    "split_candidates_examined",
    "split_candidates_beyond_trace",
    "split_search_exhaustive_vertices",
    "split_search_exhaustive_segments",
    "motorcycle_wall_tests",
    "motorcycle_march_steps",
    "motorcycle_trace_pairs",
    "motorcycle_trace_crashes",
    "motorcycle_unbounded_traces",
    "resting_steiner_vertices",
)


def measure_point(
    teeth: int, search: SplitSearch = SplitSearch.MOTORCYCLE
) -> dict:
    polygon = wavefront_corpus.comb(teeth)
    _fresh()
    started = time.perf_counter()
    skeleton = build_skeleton(polygon, split_search=search)
    elapsed = time.perf_counter() - started
    point = {
        "search": search.value,
        "vertices": polygon.vertex_count,
        "reflex": polygon.reflex_count,
        "seconds": elapsed,
        "outcome": skeleton.outcome.value,
        "nodes": len(skeleton.nodes),
        "levels": skeleton.levels,
        "sign_queries": SIGN_COUNTS["total"],
        "sign_by_conjugation": SIGN_COUNTS["closed_by_conjugation"],
    }
    for name in REPORTED_COUNTERS:
        point[name] = skeleton.counter(name)
    return point


def exponent(points: list[dict]) -> float | None:
    """Наклон log-log между первой и последней точкой. Одно число, не подгонка."""

    if len(points) < 2:
        return None
    first, last = points[0], points[-1]
    if first["seconds"] <= 0 or last["seconds"] <= 0:
        return None
    return math.log(last["seconds"] / first["seconds"]) / math.log(
        last["vertices"] / first["vertices"]
    )


def _teeth_series(arguments) -> tuple[int, ...]:
    if arguments.teeth:
        return tuple(int(value) for value in arguments.teeth.split(","))
    series = (4, 8, 16, 24, 32, 46, 62, 94, 126, 190, 254, 510)
    return tuple(
        teeth for teeth in series if 2 * teeth + 4 <= arguments.max_vertices
    )


def _report(label: str, points: list[dict]) -> float | None:
    print(f"\n=== {label} ===")
    print(
        f"{'вершин':>7} | {'reflex':>6} | {'секунд':>10} | "
        f"{'кандидатов':>10} | {'за трассой':>10} | {'перебор в/о':>11} | "
        f"{'стен':>7} | {'знаков':>8} | {'сопряж.':>7} | {'Трики, с':>9}"
    )
    for point in points:
        triki = TRIKI_SECONDS.get(point["vertices"])
        print(
            f"{point['vertices']:>7} | {point['reflex']:>6} | "
            f"{point['seconds']:>10.4f} | "
            f"{point['split_candidates_examined']:>10} | "
            f"{point['split_candidates_beyond_trace']:>10} | "
            f"{point['split_search_exhaustive_vertices']:>5}/"
            f"{point['split_search_exhaustive_segments']:<5} | "
            f"{point['motorcycle_wall_tests']:>7} | "
            f"{point['sign_queries']:>8} | "
            f"{point['sign_by_conjugation']:>7} | "
            f"{f'{triki:9.4f}' if triki else ' ' * 9}"
        )
    for earlier, later in zip(points, points[1:]):
        slope = exponent([earlier, later])
        if slope is not None:
            print(
                f"  {earlier['vertices']}->{later['vertices']}: O(n^{slope:.2f})"
            )
    slope = exponent(points)
    if slope is not None:
        print(
            f"  {points[0]['vertices']}->{points[-1]['vertices']} целиком: "
            f"O(n^{slope:.2f})"
        )
    unresolved = [point for point in points if point["outcome"] != "EXACT"]
    if unresolved:
        print(f"  НЕ EXACT: {[point['outcome'] for point in unresolved]}")
    return slope


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-vertices", type=int, default=96)
    parser.add_argument(
        "--teeth", type=str, default=None, help="через запятую, например 6,62,445,999"
    )
    parser.add_argument(
        "--search",
        choices=("motorcycle", "exhaustive", "both"),
        default="motorcycle",
    )
    parser.add_argument("--json", type=str, default=None)
    arguments = parser.parse_args()

    series = _teeth_series(arguments)
    searches = (
        (SplitSearch.MOTORCYCLE, SplitSearch.EXHAUSTIVE)
        if arguments.search == "both"
        else (SplitSearch(arguments.search.upper()),)
    )
    result: dict[str, object] = {}
    for search in searches:
        points = [measure_point(teeth, search) for teeth in series]
        slope = _report(search.value, points)
        result[search.value] = {"points": points, "exponent": slope}
    print("\nпоказатель Трики на той же форме, 42->512: O(n^2.30)")
    print("база Q1 при 2002 вершинах: 1998000 кандидатов, 25.78 с")
    if arguments.json:
        with open(arguments.json, "w", encoding="utf-8") as handle:
            json.dump(
                result, handle, ensure_ascii=False, indent=2, sort_keys=True
            )


if __name__ == "__main__":
    main()
