"""Полевые ворота восстановления канонического угла: слепок стены 2.001.

Слепок владельца лежит в дереве целиком, поэтому вопрос «а те ли это числа»
здесь не задаётся: углы вычисляются ИЗ МЕША
(`artifacts/field_snapshots/wall_2_001_snapshot.json`) точной рациональной
арифметикой по петле обхода, записанной рядом
(`wall_2_001_corner_angles.json`), и сверяются с `dot_exact` слепка побитово.
Расхождение означало бы, что тест меряет не то, что мерял владелец.

Ворота ровно два, и оба про счёт, а не про число:

1. 28 прямых углов стены — 25 точных, два чуть выше 90° и один чуть ниже —
   после восстановления дают ОДИН сертификат селекции на каждой плотности
   d0–d4 и на legacy-законе. До восстановления они расходились при чётном `q`.

2. Класс `building.003` (увод оси 0.85 мм, 3.3e-4 рад) НЕ восстанавливается и
   считается по сырому числу: допуск намерения — 7e-6 рад, и увод за ним в 48
   раз. Отрицательный контроль обязателен, иначе первое ворото доказывало бы
   только то, что канонизируется всё подряд.

Хост здесь — измерительный орган: угол рождается в
`envelope_request_export._build_angular_relations` через
`envelope_angle_certificate.certified_reflex_measure`, а решение о
восстановлении принимает ядро у двери селектора. Тест поэтому идёт хостовой
мерой и ядерным селектором, как и поле.
"""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction
import json
from pathlib import Path
import sys

import pytest
import sympy as sp


ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

SNAPSHOT_PATH = ROOT / "artifacts" / "field_snapshots" / "wall_2_001_snapshot.json"
ANGLES_PATH = (
    ROOT / "artifacts" / "field_snapshots" / "wall_2_001_corner_angles.json"
)

import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope._authoring_intent import AUTHOR_ANGULAR_ERROR  # noqa: E402
from cftuv_envelope._canonical_angle import (  # noqa: E402
    PI_RATIONAL_UPPER_BOUND,
    canonical_reflex_excess_restoration,
)
from cftuv_envelope.reference.angle_measure import (  # noqa: E402
    reflex_angle_intervals_over_pi,
)
from cftuv_envelope.reference.compile import (  # noqa: E402
    _resolve_angular_profile_selection,
    _resolve_huber_density_bucket_interval,
)

from cftuv.envelope_angle_certificate import (  # noqa: E402
    certified_reflex_measure,
)
from cftuv_envelope.reference import angle_measure  # noqa: E402


# Увод 0.85 мм на `building.003`: пара `(cos, sin)` принятого корпуса
# (`test_envelope_angle_certificate.FIELD_REFLEX_CORNERS`, второй угол).
BUILDING_003_TERMS = (-33582529823414, 10**17)

_DENSITIES = (
    (0, "LINEAR_REFLEX_DENSITY_0_V1", "PI_OVER_2", 2),
    (1, "LINEAR_REFLEX_DENSITY_1_V1", "PI_OVER_3", 3),
    (2, "LINEAR_REFLEX_DENSITY_2_V1", "PI_OVER_4", 4),
    (3, "LINEAR_REFLEX_DENSITY_3_V1", "PI_OVER_5", 5),
    (4, "LINEAR_REFLEX_DENSITY_4_V1", "PI_OVER_6", 6),
)


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _dot(left, right):
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _field_corner_terms():
    """`(вершина, класс, cos, sin²)` по всем прямым углам петли стены."""

    mesh = json.loads(SNAPSHOT_PATH.read_text(encoding="utf-8"))
    angles = json.loads(ANGLES_PATH.read_text(encoding="utf-8"))
    positions = [
        tuple(Fraction(value) for value in vertex)
        for vertex in mesh["raw"]["vertices"]
    ]
    loop = angles["loop_order"]
    recorded = {item["vertex"]: item for item in angles["corners"]}
    result = []
    for index, vertex in enumerate(loop):
        if vertex not in recorded:
            continue
        previous = loop[index - 1]
        following = loop[(index + 1) % len(loop)]
        incoming = _sub(positions[vertex], positions[previous])
        outgoing = _sub(positions[following], positions[vertex])
        cosine = _dot(incoming, outgoing)
        normal = _cross(incoming, outgoing)
        result.append(
            (
                vertex,
                recorded[vertex]["exact_vs_90"],
                cosine,
                _dot(normal, normal),
                Fraction(recorded[vertex]["dot_exact"]),
            )
        )
    return tuple(result), angles["summary"]


FIELD_CORNERS, FIELD_SUMMARY = _field_corner_terms()


def _excess_interval(cosine: Fraction, squared_sine: Fraction):
    sine = sp.sqrt(sp.Rational(squared_sine.numerator, squared_sine.denominator))
    _, excess = reflex_angle_intervals_over_pi(
        sine,
        sp.Rational(cosine.numerator, cosine.denominator),
    )
    return excess


def test_field_corner_terms_reproduce_the_owner_snapshot_bitwise():
    """Тест меряет ТЕ ЖЕ углы: `cos` совпадает с `dot_exact` слепка."""

    assert len(FIELD_CORNERS) == FIELD_SUMMARY["right_angle_corners"] == 28
    for _, _, cosine, _, recorded_dot in FIELD_CORNERS:
        assert cosine == recorded_dot
    classes = [item[1] for item in FIELD_CORNERS]
    assert classes.count("EXACTLY_90") == FIELD_SUMMARY["exactly_90"] == 25
    assert {
        vertex for vertex, kind, *_ in FIELD_CORNERS if kind == "ABOVE_90"
    } == set(FIELD_SUMMARY["above_90"])
    assert {
        vertex for vertex, kind, *_ in FIELD_CORNERS if kind == "BELOW_90"
    } == set(FIELD_SUMMARY["below_90"])


def test_field_noise_is_inside_the_declared_authoring_intent_tolerance():
    """Числа развилки: отклонение каждого шумного угла против 7e-6 рад."""

    restored = {}
    for vertex, kind, cosine, squared_sine, _ in FIELD_CORNERS:
        restoration = canonical_reflex_excess_restoration(
            _excess_interval(cosine, squared_sine)
        )
        if kind == "EXACTLY_90":
            assert restoration is None, vertex
            continue
        assert restoration is not None, vertex
        assert restoration.deviation_upper_bound_radians <= AUTHOR_ANGULAR_ERROR
        restored[vertex] = float(restoration.deviation_upper_bound_radians)
    assert set(restored) == {6, 28, 27}
    assert all(1.0e-6 < value < 2.9e-6 for value in restored.values()), restored


def test_building_003_drift_stays_raw_and_keeps_its_previous_count():
    """Отрицательный контроль: 3.3e-4 рад не восстанавливается ни на одной d."""

    cosine, sine = BUILDING_003_TERMS
    _, excess = reflex_angle_intervals_over_pi(
        sp.Integer(sine),
        sp.Integer(cosine),
    )
    deviation = max(
        abs(Fraction(excess.lower) - Fraction(1, 2)),
        abs(Fraction(excess.upper) - Fraction(1, 2)),
    )
    radians = deviation * PI_RATIONAL_UPPER_BOUND
    assert radians > AUTHOR_ANGULAR_ERROR
    assert 47 < float(radians / AUTHOR_ANGULAR_ERROR) < 49
    assert canonical_reflex_excess_restoration(excess) is None
    # Счёт у него остаётся тем, что был до карточки: на d2 — ВЕРХНЯЯ ячейка
    # (C=3, H=2), то есть ровно то, чем угол отличался от точного близнеца.
    assert _resolve_huber_density_bucket_interval(excess, 4) == 3
    # Для сравнения — ячейка канонической доли на той же плотности. Разница
    # ячеек и есть то, что восстановление даёт углам стены и НЕ даёт этому.
    assert (
        _resolve_huber_density_bucket_interval(
            _CanonicalHalf(),
            4,
        )
        == 2
    )


class _CanonicalHalf:
    """Каноническая доля π как вырожденный закрытый интервал."""

    lower = Fraction(1, 2)
    upper = Fraction(1, 2)
    lower_kind = kernel.IntervalEndpointKind.CLOSED
    upper_kind = kernel.IntervalEndpointKind.CLOSED


class _Measure:
    def __init__(self, interval):
        self.reflex_excess_over_pi = interval


@pytest.mark.parametrize(
    ("density", "density_name", "symbol_name", "q"),
    _DENSITIES,
)
def test_field_twins_share_one_selection_certificate_on_every_density(
    density,
    density_name,
    symbol_name,
    q,
):
    """Цель карточки на полевых числах: 28 углов — один сертификат селекции."""

    from cftuv.envelope_request_policy import envelope_angular_policy

    policy = envelope_angular_policy(kernel, density)
    assert policy.value_id is getattr(kernel.MaxSubturnValueId, density_name)
    base = kernel.DecalRequestV1(
        schema_version=kernel.DECAL_REQUEST_SCHEMA_V1,
        decal_request_id=kernel.DecalRequestId("field-gate"),
        selected_chain_use_ids=frozenset({kernel.ChainUseId("use")}),
        requested_alpha=kernel.LocalLengthV1(Decimal("0.25")),
        metric_space=kernel.MetricSpace.SOURCE_LOCAL_INTRINSIC,
        angular_profile_family_id=(
            kernel.AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1
        ),
        angular_profile_selection_policy_id=policy.selection_policy_id,
        max_subturn_parameter_id=policy.parameter_id,
        max_subturn_value_id=policy.value_id,
        max_subturn_exact_value=policy.exact_value,
        cap_policy_id=kernel.CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1,
        boundary_policy_id=(
            kernel.BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION
        ),
        interaction_policy_id=(
            kernel.InteractionPolicyId.INTRAPATCH_POLICY_B_V1
        ),
        ownership_policy_id=(
            kernel.OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1
        ),
        material_policy_id=kernel.PolicyId("FIELD_GATE_NO_MATERIAL_V1"),
        uv_policy_id=kernel.PolicyId("FIELD_GATE_NO_UV_V1"),
    )
    restored, raw = set(), set()
    for _, _, cosine, squared_sine, _ in FIELD_CORNERS:
        interval = _excess_interval(cosine, squared_sine)
        selection = _resolve_angular_profile_selection(base, _Measure(interval))
        assert selection is not None
        restored.add(
            (selection.hidden_count, selection.interval_certificate)
        )
        raw.add(_resolve_huber_density_bucket_interval(interval, q))
    assert len(restored) == 1, (density, restored)
    # Доказательство «тест умеет падать»: без восстановления чётные q рвутся.
    assert (len(raw) == 1) is (q % 2 == 1), (density, raw)


def test_host_measure_of_a_field_corner_feeds_the_same_restoration():
    """Мера хоста и решение ядра сходятся на одном угле, а не «примерно».

    Проверяется именно хостовая сборка `certified_reflex_measure` — та, что
    уходит в снапшот, — а не отдельно посчитанная величина.
    """

    vertex, _, cosine, squared_sine, _ = next(
        item for item in FIELD_CORNERS if item[0] == 6
    )
    measure = certified_reflex_measure(
        kernel,
        angle_measure,
        sp.sqrt(sp.Rational(squared_sine.numerator, squared_sine.denominator)),
        sp.Rational(cosine.numerator, cosine.denominator),
        kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
    )
    restoration = canonical_reflex_excess_restoration(
        measure.reflex_excess_over_pi
    )
    assert restoration is not None, vertex
    assert restoration.canonical_excess_over_pi == Fraction(1, 2)
    assert restoration.deviation_upper_bound_radians <= AUTHOR_ANGULAR_ERROR
    assert (
        measure.phi_over_pi.lower - 1 == measure.reflex_excess_over_pi.lower
    )


# --- Полевой веер: гарантия подшага на ВОССТАНОВЛЕННОМ угле -----------------
#
# Ворота цели глазами: у вершин 6 и 28 ВЫПУЩЕННЫЙ веер обязан совпасть с
# точными близнецами по счёту, числу опор и способу построения лучей. Меряется
# та самая функция, которой конвейер строит веер
# (`reference.angular._interpolated_normals`), на РЕАЛЬНЫХ опорах стены и в
# карте стены: плоскость `x = const`, поэтому чарт — это (y, z) с единичным
# Грамом, ровно как у фигуры ядра.
#
# Двухветочный закон здесь ПОВТОРЁН, а не вызван: у закона одна реализация в
# `reference.angular._canonical_subturn_fan_or_source`, и она проверена на
# уровне компиляции в `kernel/tests/test_canonical_angle_restoration.py`.
# Здесь проверяется ИСХОД на полевых числах, поэтому решение выражено теми же
# двумя предикатами ядра, что и в законе.


def _wall_chart():
    """Метрика карты стены и опоры каждого угла петли обхода."""

    import sympy as sp
    from cftuv_envelope.reference.metric import ExactPlanarMetric
    from cftuv_envelope.reference.planar_types import ExactPlanarVector

    import sys as _sys
    _sys.path.insert(0, str(ROOT / "kernel" / "tests"))
    import reference_factories as factories

    snapshot, _ = factories.angular_snapshot(1)
    metric = ExactPlanarMetric.from_descriptor(
        next(iter(snapshot.surface_metric_descriptors))
    )
    # Единичный Грам — это и есть карта плоскости `x = const`.
    assert metric.gram == ((1, 0), (0, 1))

    mesh = json.loads(SNAPSHOT_PATH.read_text(encoding="utf-8"))
    angles = json.loads(ANGLES_PATH.read_text(encoding="utf-8"))
    positions = [
        tuple(Fraction(value) for value in vertex)
        for vertex in mesh["raw"]["vertices"]
    ]
    loop = angles["loop_order"]
    recorded = {item["vertex"]: item for item in angles["corners"]}

    def vector(first: Fraction, second: Fraction):
        return ExactPlanarVector.from_values(
            sp.Rational(first.numerator, first.denominator),
            sp.Rational(second.numerator, second.denominator),
        )

    corners = []
    for index, vertex in enumerate(loop):
        if vertex not in recorded:
            continue
        previous = loop[index - 1]
        following = loop[(index + 1) % len(loop)]
        incoming = (
            positions[vertex][1] - positions[previous][1],
            positions[vertex][2] - positions[previous][2],
        )
        outgoing = (
            positions[following][1] - positions[vertex][1],
            positions[following][2] - positions[vertex][2],
        )
        # Опора перпендикулярна хорде; поворот на +90° в карте — один и тот же
        # для обеих, поэтому поворот МЕЖДУ опорами равен повороту между хордами.
        incoming_normal = vector(-incoming[1], incoming[0])
        outgoing_normal = vector(-outgoing[1], outgoing[0])
        turn = metric.oriented_cross(incoming_normal, outgoing_normal)
        orientation = (
            kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
            if sp.sign(turn) > 0
            else kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
        )
        corners.append(
            (
                vertex,
                recorded[vertex]["exact_vs_90"],
                incoming_normal,
                outgoing_normal,
                orientation,
            )
        )
    return metric, tuple(corners)


def _emitted_fan_shape(metric, corner, hidden_count, q):
    """`(профиль рациональности лучей, профиль подшагов, закон)` одного веера."""

    from cftuv_envelope.reference.angular import _interpolated_normals
    from cftuv_envelope.reference.adaptive_density_fan import _subturn
    from cftuv_envelope.reference.compile import (
        _density_ideal_is_subturn_feasible,
    )
    from cftuv_envelope.reference.direction_binding import (
        has_rational_density_support_direction,
    )

    _, _, incoming, outgoing, orientation = corner
    source = _interpolated_normals(
        metric, incoming, outgoing, hidden_count, orientation, huber_density=True
    )
    restoration = canonical_reflex_excess_restoration(
        _excess_of(metric, incoming, outgoing)
    )
    if _density_ideal_is_subturn_feasible(metric, source, q):
        law, ideal = "SOURCE", source
    elif restoration is None:
        law, ideal = "SOURCE", source
    else:
        law = "CANONICAL"
        ideal = _interpolated_normals(
            metric,
            incoming,
            outgoing,
            hidden_count,
            orientation,
            huber_density=True,
            canonical_excess_over_pi=restoration.canonical_excess_over_pi,
        )
    rational = tuple(
        has_rational_density_support_direction(metric, item) for item in ideal
    )
    subturns = tuple(
        _subturn(metric, ideal[index], ideal[index + 1], q)
        for index in range(len(ideal) - 1)
    )
    return rational, subturns, law, len(ideal)


def _excess_of(metric, incoming, outgoing):
    """Заверенный рефлексный избыток пары опор — тем же путём, что у хоста."""

    import sympy as sp

    cosine = metric.dot_g(incoming, outgoing)
    cross = metric.oriented_cross(incoming, outgoing)
    _, excess = reflex_angle_intervals_over_pi(abs(cross), cosine)
    del sp
    return excess


_HIDDEN_COUNT_BY_Q = {2: 1, 3: 1, 4: 1, 5: 2, 6: 2}


@pytest.mark.parametrize(
    ("density", "density_name", "symbol_name", "q"),
    _DENSITIES,
)
def test_field_emitted_fan_of_6_and_28_matches_the_exact_twins(
    density,
    density_name,
    symbol_name,
    q,
):
    """ГЛАВНОЕ ВОРОТО: выпущенный веер, а не только сертификат селекции.

    Сравнивается ДВА уровня, и разница между ними названа, а не замазана.

    1. Продуктовый — счёт сегментов и число опор. Он обязан совпадать с
       точными близнецами на КАЖДОЙ плотности: это и есть то, на что смотрел
       владелец («на одинаковых 270° разный счёт»).

    2. Построение лучей. Оно совпадает там, где порог тугой (`2*(H+1) == q`,
       то есть d2 и d4) — ровно там, где дефект и жил, и ровно там, где новая
       власть включается. При нетугом пороге (d0, d1, d3) старый закон
       проходит на сырых опорах у ОБОИХ углов, обещание не расходится, и
       менять построение не на что: луч остаётся сырым, байты прежние.
    """

    metric, corners = _wall_chart()
    hidden_count = _HIDDEN_COUNT_BY_Q[q]
    shapes = {}
    for corner in corners:
        vertex, kind = corner[0], corner[1]
        rational, subturns, law, support_count = _emitted_fan_shape(
            metric, corner, hidden_count, q
        )
        shapes[vertex] = (kind, rational, support_count, law)
    twin_counts = {
        item[2] for item in shapes.values() if item[0] == "EXACTLY_90"
    }
    twin_profiles = {
        item[1] for item in shapes.values() if item[0] == "EXACTLY_90"
    }
    assert len(twin_counts) == len(twin_profiles) == 1
    tight = 2 * (hidden_count + 1) == q
    for vertex in (6, 28):
        kind, rational, support_count, law = shapes[vertex]
        assert kind == "ABOVE_90"
        assert support_count == next(iter(twin_counts)), (density, q, vertex)
        if tight:
            assert law == "CANONICAL", (density, q, vertex)
            assert rational == next(iter(twin_profiles)), (density, q, vertex)
        else:
            assert law == "SOURCE", (density, q, vertex)


@pytest.mark.parametrize(
    ("density", "density_name", "symbol_name", "q"),
    _DENSITIES,
)
def test_field_below_canonical_corner_stays_on_the_source_law(
    density,
    density_name,
    symbol_name,
    q,
):
    """Вершина 27 и 25 точных углов новой власти НЕ получают."""

    metric, corners = _wall_chart()
    hidden_count = _HIDDEN_COUNT_BY_Q[q]
    for corner in corners:
        vertex, kind = corner[0], corner[1]
        if kind == "ABOVE_90":
            continue
        *_, law, _ = _emitted_fan_shape(metric, corner, hidden_count, q)
        assert law == "SOURCE", (density, q, vertex, kind)


def test_field_canonical_law_fires_exactly_where_the_threshold_is_tight():
    """Цена названа: новая власть включается только при `2*(H+1) == q`."""

    metric, corners = _wall_chart()
    fired = {}
    for _, _, _, q in ((0, 0, 0, item) for item in (2, 3, 4, 5, 6)):
        hidden_count = _HIDDEN_COUNT_BY_Q[q]
        fired[q] = {
            corner[0]
            for corner in corners
            if _emitted_fan_shape(metric, corner, hidden_count, q)[2]
            == "CANONICAL"
        }
    assert fired[2] == set() and fired[3] == set() and fired[5] == set()
    assert fired[4] == {6, 28}
    assert fired[6] == {6, 28}
