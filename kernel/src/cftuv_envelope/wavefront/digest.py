"""Семантический дайджест скелета: одно значение на одну геометрию.

Зачем отдельно. Перестановочная инвариантность — свойство, которое нельзя
заявить, его можно только сверить, и сверять нужно ЗНАЧЕНИЕ, а не объект: у
одного и того же времени в памяти лежат разные пары `(dividend, divisor)`,
потому что найдено оно разными тройками рёбер. Поэтому здесь всё приводится к
каноническому виду перед сериализацией:

- время — `EventTimeV1.canonical()`, снимающее рациональный произвол;
- точка — уже канонична: `SqrtSumV1` единственна по теореме о линейной
  независимости корней различных бесквадратных чисел;
- участники — ГЕОМЕТРИЧЕСКИЕ ключи рёбер `(a, b, c, q)`, а не индексы, иначе
  дайджест зависел бы от того, с какой вершины начат обход контура;
- порядок узлов — сортировка по самому же представлению.

Формат сериализации взят тот же, что у остального ядра: JSON с
`sort_keys=True` и компактными разделителями, sha256 от utf-8.
"""

from __future__ import annotations

from fractions import Fraction
import hashlib
import json

from .skeleton import SkeletonNodeV1, SkeletonV1
from .sqrt_sum import SqrtSumV1


def _sqrt_sum_record(value: SqrtSumV1) -> list:
    return [
        [radicand, [coefficient.numerator, coefficient.denominator]]
        for radicand, coefficient in value.terms
    ]


def _fraction_record(value: Fraction) -> list[int]:
    return [value.numerator, value.denominator]


def _participant_record(key: tuple) -> list:
    """Ключ прямой `(a, b, c, q)` в JSON. `q` бывает РАЦИОНАЛЬНЫМ.

    `a`, `b`, `c` целые всегда, `q` — только при единичной скорости и у стены.
    Дробное `q` записывается парой `[числитель, знаменатель]`, и это не выбор
    формата, а требование однозначности: `json` дробь не сериализует вовсе, а
    записать её строкой значило бы завести второе представление одного числа.

    На ЦЕЛОМ `q` запись остаётся прежней до байта, потому что `int` проходит
    мимо ветки. Ровно этим держится неподвижность `FROZEN_DIGESTS`.
    """

    return [
        _fraction_record(item) if isinstance(item, Fraction) else item
        for item in key
    ]


def node_record(node: SkeletonNodeV1) -> dict:
    """Каноническая запись узла. Ровно то, что входит в дайджест."""

    time = node.time.canonical()
    return {
        "kind": node.kind.value,
        "time_dividend": _fraction_record(time.dividend),
        "time_divisor": _sqrt_sum_record(time.divisor),
        "point_x": _sqrt_sum_record(node.point.x),
        "point_y": _sqrt_sum_record(node.point.y),
        "participants": [_participant_record(key) for key in node.participants],
        "converging_vertices": node.converging_vertices,
    }


def skeleton_records(skeleton: SkeletonV1) -> list[dict]:
    records = [node_record(node) for node in skeleton.nodes]
    records.sort(key=lambda record: json.dumps(record, sort_keys=True))
    return records


def semantic_digest(skeleton: SkeletonV1) -> str:
    """sha256 семантики скелета. Порядок входа на него влиять не должен."""

    payload = {
        "outcome": skeleton.outcome.value,
        "nodes": skeleton_records(skeleton),
    }
    encoded = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
