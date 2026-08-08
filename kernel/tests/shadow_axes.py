"""Оси наблюдаемого ответа — ОДИН набор на все теневые сверки ядра.

Зачем отдельным модулем. Теневая сверка доказывает «два пути дают побитово
один ответ», и сила такого доказательства равна ПОЛНОТЕ осей: сверка, забывшая
долги доказательства или точные площади граней, зеленеет там, где расхождение
уже есть. Две копии набора осей разойдутся ровно в тот день, когда одну из них
дополнят, — и вторые ворота молча ослабнут. Поэтому набор здесь один, и обе
сверки (`test_lazy_hydration_shadow.py`, `test_exact_identity_shadow.py`)
читают его отсюда.

Состав осей не «на глаз»: исход, semantic_digest, узлы, уровни, счётчики,
статус доказательства и ПОЛНЫЙ мультисет его долгов, грани с точными
площадями, покрытие на четырёх alpha.
"""

from __future__ import annotations

from fractions import Fraction
import json

from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record, semantic_digest


#: Alpha, на которых сверяется покрытие. Ноль и единица — границы, дробь между
#: ними — общий случай, крупная — фронт, прошедший фигуру насквозь.
ALPHAS = (Fraction(0), Fraction(1, 3), Fraction(1), Fraction(4))


def sqrt_sum_record(value) -> list:
    return [
        [radicand, [coefficient.numerator, coefficient.denominator]]
        for radicand, coefficient in value.terms
    ]


def point_record(point) -> list:
    return [sqrt_sum_record(point[0]), sqrt_sum_record(point[1])]


def obligation_record(obligation) -> dict:
    """Долг доказательства целиком, а не только его число.

    Статус — величина агрегированная, и совпасть он может при РАЗНЫХ долгах.
    Ворота, сверяющие только статус, пропустили бы подмену одного долга другим,
    и подмена эта была бы молчаливой.
    """

    return {
        name: (
            value.value if hasattr(value, "value")
            else repr(value)
        )
        for name, value in sorted(vars(obligation).items())
    } if hasattr(obligation, "__dict__") else {
        name: (
            getattr(obligation, name).value
            if hasattr(getattr(obligation, name), "value")
            else repr(getattr(obligation, name))
        )
        for name in sorted(obligation.__slots__)
    }


def skeleton_axes(skeleton) -> dict:
    return {
        "outcome": skeleton.outcome.value,
        "semantic_digest": semantic_digest(skeleton),
        "levels": skeleton.levels,
        "counters": [list(item) for item in skeleton.counters],
        "proof_status": skeleton.proof_status.value,
        "proof_obligations": [
            obligation_record(item) for item in skeleton.proof_obligations
        ],
        "nodes": sorted(
            (
                json.dumps(node_record(node), sort_keys=True)
                for node in skeleton.nodes
            )
        ),
    }


def partition_axes(partition) -> dict:
    return {
        "outcome": partition.outcome.value,
        "detail": partition.detail,
        "doubled_area": sqrt_sum_record(partition.doubled_area),
        "polygon_doubled_area": partition.polygon_doubled_area,
        "area_defect": sqrt_sum_record(partition.area_defect),
        "faces": [
            {
                "owner": repr(face.owner),
                "source_start": list(face.source_start),
                "source_end": list(face.source_end),
                "line": repr(face.line),
                "doubled_area": sqrt_sum_record(face.doubled_area),
                "points": [point_record(point) for point in face.points],
            }
            for face in partition.faces
        ],
    }


def coverage_axes(partition) -> dict:
    rows = {}
    for alpha in ALPHAS:
        covered = coverage_at(partition, alpha)
        rows[str(alpha)] = {
            "outcome": covered.outcome.value,
            "detail": covered.detail,
            "doubled_area": sqrt_sum_record(covered.doubled_area),
            "polygon_doubled_area": covered.polygon_doubled_area,
            "faces": [
                {
                    "owner": repr(face.owner),
                    "doubled_area": sqrt_sum_record(face.doubled_area),
                    "points": [point_record(point) for point in face.points],
                }
                for face in covered.faces
            ],
        }
    return rows
