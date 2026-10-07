"""Конец пролёта без места даёт названный исход, а не `TypeError`.

У вершины между антипараллельными прямыми либо сонаправленными прямыми разных
скоростей места нет по закону (`exact_candidate_view.position` отвечает `None`):
две такие прямые не пересекаются и не скользят. Вхождение `(ребро, начало,
конец)` несёт тогда `None` на месте ключа точки, и это нормальное состояние
пакета, а не ошибка: с ним проходят сотни прогонов корпуса. Падали два места,
которые считали ключ точки всегда существующим: `poststate_span._span_orientation`
итерировал `occurrence[1]`, а порядок зародышей сравнивал голые ключи, где
`None < ExactIdentityKeyV1` — `TypeError`. Тихая гибель прогона нарушает правило
«фронт либо продолжается, либо достигает именованного события, либо даёт
именованный отказ».

Класс найден синтетическим корпусом нативного порта: 17 различных многоугольников
(диагональные и масштабированные семейства, взвешенные, крест).
"""

from __future__ import annotations

import random
import re
from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import pytest

import cftuv_envelope
from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetModeV1, ExactWorkBudgetV1
from cftuv_envelope.wavefront.event_time import (
    ZERO_TIME,
    EventPointV1,
    SupportLineV1,
    compare_times,
)
from cftuv_envelope.wavefront.exact_candidate_view import (
    CandidateSpanStateV1,
    CandidateVertexStateV1,
    ExactCandidateViewV1,
)
from cftuv_envelope.wavefront.exact_identity import (
    exact_point_key,
    identity_order_key,
)
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1
from cftuv_envelope.wavefront.poststate_span import (
    PoststateSpanDisposition,
    classify_poststate_span,
)
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    ProofStatus,
    SkeletonOutcome,
    build_skeleton,
    level_budget,
    refusal_counter,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1
from cftuv_envelope.wavefront.superlevel_closure import (
    SegmentRefV1,
    SpanFamilyRefV1,
)
from wavefront_cases import named_corpus

F = Fraction
POSTSTATE = "SYMBOLIC_POSTSTATE_SPAN_AFFINE_CLASSIFICATION_UNPROVEN"
UNIFIED = "SYMBOLIC_UNIFIED_CONTACT_COMPONENT_UNRESOLVABLE"
LEFT = SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
REFUSED = SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE

# (имя, точки уже нормированной петли, `q` рёбер либо None, исход, причина отказа).
# Точки и скорости сняты с записей корпуса ПОСЛЕ нормировки ориентации, поэтому
# `PolygonV1.build` не переставляет их. Исход закреплён намеренно: порт на Rust
# сверяет свой ответ с этими именами, а улучшение движка меняет строку здесь,
# а не молча.
CORPUS_17 = (
    (
        "polyomino_weighted_a",
        ((4, 5), (4, 3), (7, 3), (7, 4), (6, 4), (6, 5), (7, 5), (7, 6),
         (6, 6), (6, 7), (4, 7), (4, 6), (3, 6), (3, 7), (0, 7), (0, 5)),
        (0, 36, 0, 0, 1, 1, 1, F(1, 4), 9, 16, 0, 0, 1, 81, 4, 4),
        LEFT,
        None,
    ),
    (
        "polyomino_weighted_b",
        ((4, 3), (5, 3), (5, 1), (6, 1), (6, 5), (4, 5), (4, 6), (6, 6),
         (6, 7), (3, 7), (3, 6), (2, 6), (2, 7), (0, 7), (0, 0), (4, 0)),
        (1, 0, 4, 0, 36, 1, 1, 9, 0, 1, 0, 1, 4, 0, 0, 9),
        REFUSED,
        UNIFIED,
    ),
    (
        "diagonal_spike_notch",
        ((0, -4), (4, 0), (0, 4), (-3, 1), (-3, -1), (-4, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "diagonal_fold",
        ((0, -2), (2, -1), (2, -2), (2, 0), (0, 0), (0, 2), (-2, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "diagonal_big_spike",
        ((0, -7), (7, 0), (0, 7), (-7, 0), (-4, -3), (-5, -2), (-3, -3)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "diagonal_fold_mirrored",
        ((0, -2), (2, -2), (2, 0), (2, -1), (0, 0), (0, 2), (-2, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "diagonal_inner_spike",
        ((0, -3), (3, 0), (2, -1), (0, 0), (0, 3), (-3, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "diagonal_bottom_spike",
        ((0, -5), (-1, -4), (1, -4), (5, 0), (3, 3), (0, 5), (-5, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "scaled_axis_spike",
        ((0, -24), (0, -8), (0, -16), (24, 0), (16, 0), (0, 24), (-24, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "scaled_huge_notch",
        ((0, -4194304), (4194304, 0), (0, 4194304), (-2097152, 2097152),
         (-1048576, 3145728), (-4194304, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "scaled_fold",
        ((0, -256), (128, 0), (0, 0), (256, 0), (0, 256), (-256, 256),
         (-256, 0)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "scaled_big_spike",
        ((0, -10240), (10240, 0), (0, 10240), (-8192, 2048), (-12288, -2048),
         (-10240, 0), (-8192, -8192)),
        None,
        REFUSED,
        POSTSTATE,
    ),
    (
        "histogram_weighted",
        ((0, 0), (51, 0), (51, 3), (39, 3), (39, 15), (30, 15), (30, 6),
         (27, 6), (27, 15), (15, 15), (15, 12), (9, 12), (9, 9), (0, 9)),
        (2601, 0, 144, 0, F(81, 4), 0, 81, 0, 144, 0, 36, 81, 0, 0),
        LEFT,
        None,
    ),
    (
        "cross_sources",
        ((0, -1), (4, -1), (4, 0), (10, 0), (10, 3), (4, 3), (4, 9), (0, 9),
         (0, 3), (-2, 3), (-2, 0), (0, 0)),
        (16, 1, 36, 0, 0, 0, 0, 0, 4, 9, 0, 1),
        REFUSED,
        POSTSTATE,
    ),
    (
        "cross_weighted_a",
        ((0, -3), (3, -3), (3, 0), (11, 0), (11, 6), (3, 6), (3, 8), (0, 8),
         (0, 6), (-8, 6), (-8, 0), (0, 0)),
        (0, 0, 64, 144, 0, 0, 9, 1, 64, 144, 576, F(9, 4)),
        REFUSED,
        POSTSTATE,
    ),
    (
        "cross_weighted_b",
        ((0, -4), (3, -4), (3, 0), (8, 0), (8, 5), (3, 5), (3, 11), (0, 11),
         (0, 5), (-2, 5), (-2, 0), (0, 0)),
        (9, 0, 100, 0, 0, 36, 0, 0, 16, 25, 0, 4),
        REFUSED,
        UNIFIED,
    ),
    (
        "diagonal_sources",
        ((0, -9), (5, -4), (9, 0), (0, 9), (-6, 3), (-9, 0)),
        (50, 0, 162, 0, 0, 162),
        REFUSED,
        POSTSTATE,
    ),
)

MINIMAL_REPRO = ((0, -4), (4, 0), (0, 4), (-3, 1), (-3, -1), (-4, 0))


def _polygon(points, speeds) -> PolygonV1:
    return PolygonV1.build(LoopV1(points, speeds))


def _bounded_budget() -> ExactWorkBudgetV1:
    return ExactWorkBudgetV1(
        mode=ExactWorkBudgetModeV1.BOUNDED, cap=10**9, stage="TEST"
    )


def validate_skeleton(polygon: PolygonV1, skeleton) -> None:
    """Проверки результата: две объявленные границы, именованность отказа, грани.

    Граница 1 — время узлов не убывает; граница 2 — уровней не больше
    `level_budget`. Скелет, не ставший `EXACT`, обязан нести хотя бы один
    названный долг и неполный статус доказательства: исход без имени — это
    тихое исчезновение. `EXACT` обязан иметь полное доказательство и разбиение
    на грани с тремя выполненными границами.
    """

    times = [node.time for node in skeleton.nodes]
    assert all(
        compare_times(left, right) <= 0 for left, right in zip(times, times[1:])
    ), "время узлов убывает"
    assert skeleton.levels <= level_budget(polygon)
    if skeleton.outcome is SkeletonOutcome.EXACT:
        assert skeleton.proof_status is ProofStatus.COMPLETE
        partition = build_faces(polygon, skeleton)
        assert partition.outcome is FaceOutcome.EXACT
        assert partition.every_contour_is_simple
        assert partition.area_reproduces_polygon
        return
    assert skeleton.proof_status is ProofStatus.INCOMPLETE
    assert skeleton.proof_obligations, "отказ без названного долга"
    if skeleton.outcome is SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE:
        assert any(
            name.startswith("superlevel_unresolvable_reason::") and value
            for name, value in skeleton.counters
        ), "отказ пакета без названной причины"


# --------------------------------------------------------------------------
# 1. Минимальный повтор и весь класс
# --------------------------------------------------------------------------


def test_the_minimal_diagonal_repro_is_a_named_refusal_with_and_without_a_budget():
    polygon = PolygonV1.build(MINIMAL_REPRO)
    answers = []
    for budget in (None, _bounded_budget()):
        skeleton = build_skeleton(polygon, work_budget=budget)
        validate_skeleton(polygon, skeleton)
        assert skeleton.outcome is REFUSED
        assert skeleton.counter(f"superlevel_unresolvable_reason::{POSTSTATE}") == 1
        # Вершина (-4, 0) стоит между антипараллельными рёбрами: места у неё нет,
        # и событийный цикл называет это в тот же прогон.
        assert skeleton.counter(
            refusal_counter(CandidateRefusal.NO_RULE_JOINT_IS_ANTIPARALLEL)
        ) == 1
        answers.append((skeleton.outcome, skeleton.levels, skeleton.counters))
    assert answers[0] == answers[1], "бюджет меняет цену, а не ответ"


@pytest.mark.parametrize("bounded", (False, True), ids=("no_budget", "budget"))
@pytest.mark.parametrize(
    "name,points,speeds,outcome,reason",
    CORPUS_17,
    ids=[item[0] for item in CORPUS_17],
)
def test_every_polygon_of_the_class_gets_a_validated_named_answer(
    name, points, speeds, outcome, reason, bounded
):
    polygon = _polygon(points, speeds)
    skeleton = build_skeleton(
        polygon, work_budget=_bounded_budget() if bounded else None
    )
    validate_skeleton(polygon, skeleton)
    assert skeleton.outcome is outcome
    if reason is not None:
        assert skeleton.counter(f"superlevel_unresolvable_reason::{reason}") == 1


def test_the_corpus_has_seventeen_distinct_polygons():
    loops = [_polygon(points, speeds).outer for _, points, speeds, _, _ in CORPUS_17]
    keys = {(loop.points, loop.speeds_squared) for loop in loops}
    assert len(CORPUS_17) == len(keys) == 17


def test_the_validator_is_not_vacuous():
    polygon = _polygon(CORPUS_17[0][1], CORPUS_17[0][2])
    skeleton = build_skeleton(polygon)
    assert len(skeleton.nodes) > 2
    validate_skeleton(polygon, skeleton)
    with pytest.raises(AssertionError):
        validate_skeleton(
            polygon, replace(skeleton, nodes=tuple(reversed(skeleton.nodes)))
        )
    with pytest.raises(AssertionError):
        validate_skeleton(polygon, replace(skeleton, proof_obligations=()))
    with pytest.raises(AssertionError):
        validate_skeleton(polygon, replace(skeleton, levels=10**6))


EXACT_CONTROLS = tuple(
    item
    for item in named_corpus()
    if item[0] in {"axis_square", "diamond", "ell", "comb_2"}
)


@pytest.mark.parametrize(
    "name,polygon", EXACT_CONTROLS, ids=[item[0] for item in EXACT_CONTROLS]
)
def test_the_validator_accepts_exact_skeletons_of_valid_polygons(name, polygon):
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    validate_skeleton(polygon, skeleton)


# --------------------------------------------------------------------------
# 2. Ориентация пролёта, чей конец без места
# --------------------------------------------------------------------------


def _line(start, end, speed=1):
    return SupportLineV1.with_speed(start, end, Fraction(speed))


def _point_key(x, y):
    return exact_point_key(
        EventPointV1(SqrtSumV1.rational(x), SqrtSumV1.rational(y))
    )


def _view_with_shared_occurrence(start, end):
    occurrence = ((0, 0, 10, 0), start, end)
    shared = SegmentRefV1(
        SpanFamilyRefV1(occurrence, (occurrence[0],)), None, None, occurrence
    )
    spans = {
        "low": CandidateSpanStateV1(
            _line((0, 1), (0, 0)), (0, 0, 0, 1), None, None
        ),
        shared: CandidateSpanStateV1(
            _line((0, 0), (10, 0), 0), (0, 0, 10, 0), "low", "high"
        ),
        "high": CandidateSpanStateV1(
            _line((10, 0), (10, 1)), (10, 0, 10, 1), None, None
        ),
    }
    vertices = {
        "low": CandidateVertexStateV1("low", shared, ZERO_TIME, None),
        "high": CandidateVertexStateV1(shared, "high", ZERO_TIME, None),
    }
    return ExactCandidateViewV1(
        (), vertices.__getitem__, spans.__getitem__, lambda *_: None
    )


def test_an_occurrence_end_without_a_place_leaves_the_span_orientation_unproven():
    for start, end in (
        (None, _point_key(10, 0)),
        (_point_key(0, 0), None),
        (None, None),
    ):
        result = classify_poststate_span(
            _view_with_shared_occurrence(start, end), "low", "high", ZERO_TIME
        )
        assert result.disposition is (
            PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN
        )
        assert result.orientation_sign is None
        assert result.birth_length is not None


def test_an_occurrence_with_both_places_keeps_its_orientation_law():
    forward = classify_poststate_span(
        _view_with_shared_occurrence(_point_key(0, 0), _point_key(10, 0)),
        "low",
        "high",
        ZERO_TIME,
    )
    backward = classify_poststate_span(
        _view_with_shared_occurrence(_point_key(10, 0), _point_key(0, 0)),
        "low",
        "high",
        ZERO_TIME,
    )
    assert forward.orientation_sign == 1
    assert backward.orientation_sign == -1


# --------------------------------------------------------------------------
# 3. Порядок ключей с отсутствующим концом
# --------------------------------------------------------------------------


def _random_key(rng, depth=0):
    if depth >= 3 or rng.random() < 0.35:
        return rng.choice((None, 0, 1, 2, F(1, 2), F(-3, 4), False, True))
    return tuple(_random_key(rng, depth + 1) for _ in range(rng.randint(1, 3)))


def _shape(value):
    """Форма ключа без значений листьев: сравнивать можно только одинаковые формы."""

    if isinstance(value, tuple):
        return tuple(_shape(item) for item in value)
    return None


def test_identity_order_key_keeps_the_order_of_keys_without_none():
    rng = random.Random(20261007)
    keys = [
        (
            rng.randint(-3, 3),
            ((rng.randint(0, 2), F(rng.randint(-5, 5), rng.randint(1, 4))),),
            (rng.randint(0, 1), rng.randint(0, 1)),
        )
        for _ in range(400)
    ]
    assert sorted(keys, key=identity_order_key) == sorted(keys)


def test_identity_order_key_puts_none_after_every_value_of_the_same_slot_without_failing():
    plain = ("edge", ((1, F(2)),), ((1, F(3)),))
    cut = ("edge", None, ((1, F(3)),))
    ordered = sorted([cut, plain], key=identity_order_key)
    assert ordered == [plain, cut]
    with pytest.raises(TypeError):
        sorted([cut, plain])
    both = ("edge", None, None)
    assert sorted([both, cut, plain], key=identity_order_key) == [plain, cut, both]


def test_identity_order_key_is_a_total_order_over_keys_with_none():
    rng = random.Random(7)
    pool = []
    while len(pool) < 60:
        key = _random_key(rng)
        if isinstance(key, tuple):
            pool.append(key)
    by_shape = {}
    for key in pool:
        by_shape.setdefault(_shape(key), []).append(key)
    assert any(len(group) > 1 for group in by_shape.values())
    for group in by_shape.values():
        for _ in range(5):
            shuffled = list(group)
            rng.shuffle(shuffled)
            assert sorted(shuffled, key=identity_order_key) == sorted(
                group, key=identity_order_key
            )


# --------------------------------------------------------------------------
# 4. Ни одного голого сравнения ключей зародышей
# --------------------------------------------------------------------------

_RAW_BIRTH_ORDER = re.compile(
    r"key\s*=\s*lambda\s+\w+\s*:\s*\w+\.key\s*[,)]"
    r"|sorted\(\s*\(\s*remapped\[key\]"
)


def test_no_wavefront_module_orders_birth_keys_by_bare_comparison():
    """Ключ зародыша несёт вхождения, а в вхождении конец без места — `None`.

    Сортировка `key=lambda item: item.key` падает, как только два зародыша
    одного времени и точки различаются только тем, есть ли у конца место.
    Порядок обязан идти через `identity_order_key`.
    """

    wavefront = Path(cftuv_envelope.__file__).resolve().parent / "wavefront"
    offenders = []
    for path in sorted(wavefront.glob("*.py")):
        text = path.read_text(encoding="utf-8")
        for match in _RAW_BIRTH_ORDER.finditer(text):
            offenders.append(f"{path.name}:{text.count(chr(10), 0, match.start()) + 1}")
    assert not offenders, offenders
