"""Зеркальная пара: фигура и её отражение обязаны дать ОДИН исход.

Полевой класс «зеркальные окна съезжают» проверяется здесь напрямую: отражение
`x -> -x` — изометрия, она не меняет ни одной геометрической величины задачи.
Любое расхождение исходов между фигурой и её зеркалом означает, что ответ
зависит от ХИРАЛЬНОСТИ, то есть от порядка, взятого не из геометрии.

Запуск (из каталога kernel):

    PYTHONPATH=src:tests python3 artifacts/p0_2_superlevel_transaction/\
        probe_mirror_covariance.py

Измерение на вершине exec/p0-2b-finish: 15 из 86 фигур корпуса АСИММЕТРИЧНЫ.
Самая громкая — `weighted::weighted_vertex_fan`:
SUPERLEVEL_COMPONENT_UNRESOLVABLE (SYMBOLIC_F0_OVERLAY_UNRESOLVABLE) в одной
хиральности и EXACT в зеркальной. Отказ приходит ДО всякой классификации
poststate, поэтому `_span_orientation` тут ни при чём: с ориентацией,
принудительно равной +1, число асимметричных фигур то же самое — 15.
"""
import collections, json, sys
from dataclasses import replace
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1
from cftuv_envelope.wavefront.skeleton import build_skeleton
from cftuv_envelope.wavefront.faces import build_faces
import os
from cftuv_envelope.wavefront import poststate_span as _PS
if os.environ.get("FORCE_ORIENT"):
    # Контроль: ориентация пролёта не участвует. Число асимметрий не меняется.
    _PS._span_orientation = lambda view, ref: 1
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus


def mirrored(polygon):
    """Отражение x -> -x. Скорости привязаны к рёбрам, `oriented` вернёт обход."""

    def flip(loop):
        return LoopV1(
            tuple((-x, y) for x, y in loop.points), loop.speeds_squared
        )

    return PolygonV1.build(flip(polygon.outer), tuple(map(flip, polygon.holes)))


def axes(polygon):
    sk = build_skeleton(polygon)
    partition = build_faces(polygon, sk)
    reasons = tuple(sorted(
        (k, v) for k, v in sk.counters if "unresolvable_reason" in k
    ))
    return (sk.outcome.value, sk.levels, len(sk.nodes), len(partition.faces),
            partition.outcome.value, sk.proof_status.value, reasons)


cases = [(f"named::{n}", p) for n, p in (*named_corpus(), *partial_source_corpus())]
cases += [(f"weighted::{c.name}", c.polygon) for c in weighted_wall_differential_corpus()]
bad = []
stats = collections.Counter()
for name, polygon in cases:
    try:
        other = mirrored(polygon)
    except Exception as error:
        stats["mirror_build_failed"] += 1
        continue
    left, right = axes(polygon), axes(other)
    stats["compared"] += 1
    if left != right:
        stats["ASYMMETRIC"] += 1
        bad.append((name, left, right))
print(json.dumps({str(k): v for k, v in stats.items()}, indent=1))
for name, left, right in bad[:14]:
    print(" ", name)
    print("     прямая ", left)
    print("     зеркало", right)
