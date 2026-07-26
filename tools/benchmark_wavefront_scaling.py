"""Масштабирование очереди событий: время построения от числа вершин.

Против чего меряем. `mesh_inset` Говарда Трики на той же гребёнке с reflex-
зубьями (замер записан в `DECISIONS.md`, 2026-07-25):

    42 -> 0.0084 c, 128 -> 0.1288 c, 256 -> 0.5455 c,
    512 -> 2.6185 c, 1024 -> 100.89 c;  показатель 42->512 = O(n^2.30)

Его код — binary64 с допусками и поиском edge-событий перебором всех спиц;
наш — точная целочисленная арифметика и наивный поиск split-кандидатов. То
есть сравниваются РАЗНЫЕ задачи, и стенд обязан показывать не «кто быстрее», а
ПОКАЗАТЕЛЬ СТЕПЕНИ: он и говорит, упирается ли очередь в тот же O(n^2+), ради
которого нужен motorcycle graph.

Каждая точка меряется на СВЕЖИХ значениях: многоугольник строится заново, а
кеши `squarefree_split`/`prime_support` сбрасываются. Без сброса второй прогон
мерил бы факторизацию из кеша, а не арифметику.

Запуск:
    PYTHONPATH=kernel/src:tools python3 tools/benchmark_wavefront_scaling.py
    PYTHONPATH=kernel/src:tools python3 tools/benchmark_wavefront_scaling.py --max-vertices 128
"""

from __future__ import annotations

import argparse
import json
import math
import time

from cftuv_envelope.wavefront.skeleton import build_skeleton
from cftuv_envelope.wavefront.sqrt_sum import (
    SIGN_COUNTS,
    prime_support,
    reset_sign_counts,
    squarefree_split,
)

import wavefront_corpus


# Замеры Трики на той же форме, из `DECISIONS.md`. Не для соревнования, а для
# того, чтобы показатель степени было с чем сопоставить.
TRIKI_SECONDS = {42: 0.0084, 128: 0.1288, 256: 0.5455, 512: 2.6185, 1024: 100.89}


def _fresh() -> None:
    squarefree_split.cache_clear()
    prime_support.cache_clear()
    reset_sign_counts()


def measure_point(teeth: int) -> dict:
    polygon = wavefront_corpus.comb(teeth)
    _fresh()
    started = time.perf_counter()
    skeleton = build_skeleton(polygon)
    elapsed = time.perf_counter() - started
    return {
        "vertices": polygon.vertex_count,
        "reflex": polygon.reflex_count,
        "seconds": elapsed,
        "outcome": skeleton.outcome.value,
        "nodes": len(skeleton.nodes),
        "levels": skeleton.levels,
        "split_candidates_examined": skeleton.counter("split_candidates_examined"),
        "sign_queries": SIGN_COUNTS["total"],
        "sign_by_conjugation": SIGN_COUNTS["closed_by_conjugation"],
    }


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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-vertices", type=int, default=96)
    parser.add_argument("--json", type=str, default=None)
    arguments = parser.parse_args()

    points: list[dict] = []
    for teeth in (4, 8, 16, 24, 32, 46, 62, 94, 126, 190, 254, 510):
        vertices = 2 * teeth + 4
        if vertices > arguments.max_vertices:
            break
        points.append(measure_point(teeth))

    print(
        f"{'вершин':>7} | {'reflex':>6} | {'секунд':>10} | {'узлов':>6} | "
        f"{'уровней':>7} | {'кандидатов':>10} | {'знаков':>8} | {'Трики, с':>9}"
    )
    for point in points:
        triki = TRIKI_SECONDS.get(point["vertices"])
        triki_text = f"{triki:9.4f}" if triki else " " * 9
        print(
            f"{point['vertices']:>7} | {point['reflex']:>6} | "
            f"{point['seconds']:>10.4f} | {point['nodes']:>6} | "
            f"{point['levels']:>7} | {point['split_candidates_examined']:>10} | "
            f"{point['sign_queries']:>8} | {triki_text}"
        )
    slope = exponent(points)
    if slope is not None:
        print(
            f"\nпоказатель log-log {points[0]['vertices']}->"
            f"{points[-1]['vertices']}: O(n^{slope:.2f})"
        )
    print("показатель Трики на той же форме, 42->512: O(n^2.30)")
    unresolved = [point for point in points if point["outcome"] != "EXACT"]
    if unresolved:
        print(f"НЕ EXACT: {[point['outcome'] for point in unresolved]}")
    if arguments.json:
        with open(arguments.json, "w", encoding="utf-8") as handle:
            json.dump(
                {"points": points, "exponent": slope},
                handle,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )


if __name__ == "__main__":
    main()
