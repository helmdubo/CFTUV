"""Зеркальная пара: фигура и её отражение обязаны дать один исход."""
import collections, json, sys
from dataclasses import replace
from cftuv_envelope.wavefront.polygon import (
    FanSupportV1, LoopV1, PolygonV1, VertexFanV1)
from cftuv_envelope.wavefront.skeleton import build_skeleton
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront import poststate_span as _PS
import os
if os.environ.get('FORCE_ORIENT'):
    _PS._span_orientation = lambda view, ref: 1
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus


def mirrored(polygon):
    """Отражение `x -> -x` со ВСЕМ входом: контуры, скорости и вееры.

    Веер отражается вместе с фигурой: точка меняет знак x, нормаль каждой
    скрытой опоры — тоже (`(a, b) -> (-a, b)`), а ПОРЯДОК опор
    разворачивается, потому что отражение меняет знак обхода и роли входящего
    и исходящего рёбер вершины меняются местами. Забыть вееры значит сравнить
    фигуру не с её зеркалом, а с другой фигурой.
    """

    def flip_loop(loop):
        return LoopV1(
            tuple((-x, y) for x, y in loop.points), loop.speeds_squared
        ).oriented(counter_clockwise=loop.is_counter_clockwise)

    def flip_fan(fan):
        return VertexFanV1(
            (-fan.point[0], fan.point[1]),
            tuple(
                FanSupportV1(-item.normal_x, item.normal_y, item.speed_squared)
                for item in reversed(fan.supports)
            ),
        )

    return PolygonV1(
        flip_loop(polygon.outer),
        tuple(flip_loop(hole) for hole in polygon.holes),
        tuple(flip_fan(fan) for fan in polygon.vertex_fans),
    )


def axes(polygon):
    """Оси, по которым отражение обязано совпасть ТОЧНО.

    Исход, доказательства и разрешимость — содержимое ответа. Число уровней
    дренажа — машинерия, отражению не обязана; порядок записей и знаки —
    представление.
    """

    sk = build_skeleton(polygon)
    partition = build_faces(polygon, sk)
    reasons = tuple(sorted(
        (k, v) for k, v in sk.counters if "unresolvable_reason" in k
    ))
    strict = (sk.outcome.value, partition.outcome.value,
              sk.proof_status.value, reasons)
    content = (len(sk.nodes), len(partition.faces))
    return strict, content


cases = [(f"named::{n}", p) for n, p in (*named_corpus(), *partial_source_corpus())]
cases += [(f"weighted::{c.name}", c.polygon) for c in weighted_wall_differential_corpus()]
bad = []
stats = collections.Counter()
for name, polygon in cases:
    try:
        other = mirrored(polygon)
    except Exception as error:
        stats["mirror_build_failed"] += 1
        if stats["mirror_build_failed"] == 1:
            import traceback; traceback.print_exc()
        continue
    left, right = axes(polygon), axes(other)
    stats["compared"] += 1
    if left[0] != right[0]:
        stats["ASYMMETRIC_STRICT(outcome/proof/разрешимость)"] += 1
        bad.append((name, left, right))
    elif left[1] != right[1]:
        stats["ASYMMETRIC_CONTENT(узлы/грани)"] += 1
        bad.append((name, left, right))
print(json.dumps({str(k): v for k, v in stats.items()}, indent=1))
for name, left, right in bad:
    kind = "STRICT" if left[0] != right[0] else "content"
    if kind == "STRICT" or "-v" in __import__("sys").argv:
        print(" ", kind, name)
        print("     прямая ", left)
        print("     зеркало", right)
