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
from .events import EventKind
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
    record = {
        "kind": node.kind.value,
        "time_dividend": _fraction_record(time.dividend),
        "time_divisor": _sqrt_sum_record(time.divisor),
        "point_x": _sqrt_sum_record(node.point.x),
        "point_y": _sqrt_sum_record(node.point.y),
        "participants": [_participant_record(key) for key in node.participants],
        "converging_vertices": node.converging_vertices,
    }
    if node.kind is EventKind.MULTIWAY:
        canonical_kinds = tuple(
            sorted(set(node.kinds), key=lambda kind: kind.value)
        )
        if not canonical_kinds:
            raise ValueError("MULTIWAY_NODE_KINDS_UNAVAILABLE")
        if EventKind.MULTIWAY in canonical_kinds or node.kinds != canonical_kinds:
            raise ValueError("MULTIWAY_NODE_KINDS_NOT_CANONICAL")
        if not node.incidences:
            raise ValueError("MULTIWAY_NODE_INCIDENCE_UNAVAILABLE")
        canonical_incidences = tuple(
            sorted(
                tuple(sorted(set(incidence))) for incidence in node.incidences
            )
        )
        if node.incidences != canonical_incidences:
            raise ValueError("MULTIWAY_NODE_INCIDENCES_NOT_CANONICAL")
        incidence_union = tuple(
            sorted(
                {
                    participant
                    for incidence in node.incidences
                    for participant in incidence
                }
            )
        )
        if incidence_union != node.participants:
            raise ValueError("MULTIWAY_NODE_INCIDENCE_UNION_MISMATCH")
        record["kinds"] = [kind.value for kind in canonical_kinds]
        record["incidences"] = [
            [_participant_record(key) for key in incidence]
            for incidence in canonical_incidences
        ]
    return record


def skeleton_records(skeleton: SkeletonV1) -> list[dict]:
    records = [node_record(node) for node in skeleton.nodes]
    records.sort(key=lambda record: json.dumps(record, sort_keys=True))
    return records


def duplicate_node_counts(nodes: tuple[SkeletonNodeV1, ...]) -> tuple[int, int]:
    """Лишние canonical `(time, point)` records и mixed-kind места."""

    groups: dict[str, list[str]] = {}
    for node in nodes:
        record = node_record(node)
        key = json.dumps(
            [
                record["time_dividend"],
                record["time_divisor"],
                record["point_x"],
                record["point_y"],
            ],
            sort_keys=True,
            separators=(",", ":"),
        )
        groups.setdefault(key, []).append(record["kind"])
    duplicates = sum(max(0, len(kinds) - 1) for kinds in groups.values())
    mixed = sum(len(set(kinds)) > 1 for kinds in groups.values())
    return duplicates, mixed


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
