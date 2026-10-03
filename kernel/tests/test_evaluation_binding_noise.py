"""FAN-CANONICAL-COUNT: шум привязки к решётке не решает счёт канонического угла.

Полевой дефект (`artifacts/fan_consistency`): точный прямой вогнутый угол
(доля `1/2`) на тугом пороге d2/d4 (`u*q == H+1`) получал разный счёт и разные
веера в зависимости от ЗНАКА шума привязки вершин к решётке в косой карте.
Закон `EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1` переносит решение на
канонический веер и называет всё, что изменил: знак и `cos^2` шума и точную
границу боковых смещений привязки — в записи, которую независимый проверяющий
пересчитывает по сырой геометрии.

Данные — выгрузка живой сцены владельца (`kernel/fixtures/binding_noise_canonical_v1`):
`building_patch114` несёт по углу каждого знака шума (ниже и выше 90 градусов),
`building_patch4` — угол выше, `building004_patch0` — честный околопрямой угол
90.56 градуса (не канон: закон молчит). Нулевой шум — `density4_exact_limit_v1`.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import (
    AngularEnvelopeSpec,
    EvaluationGeometrySubturnCountLiftLawV1,
    ExactRatioV1,
    ExactTurnSignV1,
)
from cftuv_envelope._canonical_angle import (
    canonical_count_is_tight,
    exact_canonical_selector_fact,
)
from cftuv_envelope.ids import SelectionCertificateId
from cftuv_envelope._density_policy import (
    EVALUATION_SUBTURN_LIFT_PREDICATES,
    huber_density_value_contract,
)
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.numeric import IntervalEndpointKind
from cftuv_envelope.reference import ReferenceOutcome, compile_reference_envelopes
from cftuv_envelope.reference import evaluation_binding_noise as noise
from cftuv_envelope.reference.angular import (
    _ideal_angular_support_data,
    _incident_normal,
    _interpolated_normals,
    seal_angular_support_cache,
)
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.compile import (
    _density_ideal_is_subturn_feasible,
    _exact_turn_witness,
)
from cftuv_envelope.reference.contracts import (
    EvaluationBindingNoiseEffectV1,
    EvaluationBindingNoiseLawV1,
    EvaluationBindingNoiseRefusalV1,
)
from cftuv_envelope.reference.planar_types import ExactPlanarVector
from cftuv_envelope.reference.validation import validate_reference_geometry_payload

KERNEL = Path(__file__).resolve().parents[1]
FIXTURE = KERNEL / "fixtures" / "binding_noise_canonical_v1"
EXACT_FIXTURE = KERNEL / "fixtures" / "density4_exact_limit_v1"

CANONICAL_LIFT = (
    EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1
)
STRICT = EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1
EXACT_LIMIT = (
    EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1
)
FAN = EvaluationBindingNoiseEffectV1.CANONICAL_ROTATION_FAN
LIFT = EvaluationBindingNoiseEffectV1.CANONICAL_COUNT_LIFT

#: Счёт канонического (прямого) угла: d2 держит один скрытый луч, d4 — три.
CANONICAL_COUNT = {2: 1, 4: 3}


def _load(folder: Path, name: str, density: int):
    base = folder / name
    return (
        kernel.AnalysisSnapshotCodecV1.loads(
            (base / "analysis_snapshot.json").read_bytes()
        ),
        kernel.DecalRequestCodecV1.loads(
            (base / f"decal_request_density{density}.json").read_bytes()
        ),
    )


@pytest.fixture(scope="module")
def compiled():
    cache = {}

    def get(name: str, density: int, folder: Path = FIXTURE):
        key = (folder, name, density)
        if key not in cache:
            snapshot, request = _load(folder, name, density)
            result = compile_reference_envelopes(snapshot, request)
            assert result.outcome is ReferenceOutcome.EXACT, (name, density, result.diagnostics)
            cache[key] = (snapshot, result.compilation)
        return cache[key]

    return get


def _specs(compilation):
    return sorted(
        (
            item
            for item in compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
        ),
        key=lambda item: item.envelope_spec_id.value,
    )


def _context(snapshot, compilation):
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    return GeometryContext.build(compilation, frame), frame


def _lift(spec):
    return getattr(spec, "evaluation_subturn_count_lift", None)


def _noise_sign(context, spec) -> int:
    """Знак шума: `+1` — угол ВЫШЕ 90 градусов, `-1` — ниже, `0` — точный прямой.

    Считается по привязанным координатам и Граму карты заново, не по записям.
    """

    relation, sector = noise._corner_ids(context, spec)
    previous, vertex, following = noise._corner_vertices(context, relation, sector)
    coordinates = {
        item.source_vertex_id: (
            Fraction(item.domain_coordinate.x.numerator, item.domain_coordinate.x.denominator),
            Fraction(item.domain_coordinate.y.numerator, item.domain_coordinate.y.denominator),
        )
        for item in context.compilation.evaluation_geometry_binding.source_vertex_coordinates
    }
    g00, g01, g11 = noise._gram(context.frame)
    incoming = tuple(b - a for a, b in zip(coordinates[previous], coordinates[vertex]))
    outgoing = tuple(b - a for a, b in zip(coordinates[vertex], coordinates[following]))
    dot = (
        incoming[0] * outgoing[0] * g00
        + (incoming[0] * outgoing[1] + incoming[1] * outgoing[0]) * g01
        + incoming[1] * outgoing[1] * g11
    )
    return (dot < 0) - (dot > 0)


def _classes(snapshot, compilation):
    context, _ = _context(snapshot, compilation)
    return context, {spec.envelope_spec_id: _noise_sign(context, spec) for spec in _specs(compilation)}


# --------------------------------------------------------------------------
# 1. Один счёт при любом знаке шума
# --------------------------------------------------------------------------


@pytest.mark.parametrize("density", (2, 4))
def test_a_right_angle_gets_one_count_whatever_the_sign_of_the_binding_noise(
    compiled, density
):
    """Выше, ниже и точно: один счёт и ни одного расхождения между углами.

    Фикстура несёт ОБА знака шума (это проверяется, а не предполагается), а
    точный нуль берётся из выгрузки `density4_exact_limit_v1`.
    """

    snapshot, compilation = compiled("building_patch114", density)
    context, signs = _classes(snapshot, compilation)
    assert set(signs.values()) == {-1, 1}
    counts = {spec.resolved_hidden_edge_count for spec in _specs(compilation)}
    assert counts == {CANONICAL_COUNT[density]}

    exact_snapshot, exact_compilation = compiled(
        "mesh2_patch2", density, EXACT_FIXTURE
    )
    _, exact_signs = _classes(exact_snapshot, exact_compilation)
    assert set(exact_signs.values()) == {0}
    assert {
        spec.resolved_hidden_edge_count for spec in _specs(exact_compilation)
    } == {CANONICAL_COUNT[density]}


@pytest.mark.parametrize("name", ("building_patch114", "building_patch4"))
@pytest.mark.parametrize("density", (2, 4))
def test_the_independent_verifier_accepts_every_corner_consistently(
    compiled, name, density
):
    """`GeometryContext.build` и печать опор принимают весь домен, оба знака."""

    snapshot, compilation = compiled(name, density)
    context, _ = _context(snapshot, compilation)
    seal_angular_support_cache(context)
    assert compilation.canonical_subturn_fan_authorities == frozenset()


# --------------------------------------------------------------------------
# 2. Что именно записано: закон, исход, точные числа
# --------------------------------------------------------------------------


def _above_and_below(snapshot, compilation):
    """Единственный угол выше 90 градусов и единственный угол ниже (фикстура 114)."""

    context, signs = _classes(snapshot, compilation)
    above = [spec for spec in _specs(compilation) if signs[spec.envelope_spec_id] > 0]
    below = [spec for spec in _specs(compilation) if signs[spec.envelope_spec_id] < 0]
    assert len(above) == len(below) == 1
    return context, above[0], below[0]


def test_d2_the_above_corner_gets_the_canonical_fan_and_a_named_record(compiled):
    snapshot, compilation = compiled("building_patch114", 2)
    context, above, below = _above_and_below(snapshot, compilation)
    # Угол ВЫШЕ 90 ломал гарантию шага на вычислительном веере.
    assert above.resolved_hidden_edge_count == below.resolved_hidden_edge_count == 1
    assert _lift(above) is None and _lift(below) is None
    records = {
        item.envelope_spec_id: item
        for item in compilation.evaluation_binding_noise_records
    }
    assert set(records) == {above.envelope_spec_id}
    record = records[above.envelope_spec_id]
    assert record.noise_law is EvaluationBindingNoiseLawV1.EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1
    assert record.effect is FAN
    assert (record.source_hidden_edge_count, record.effective_hidden_edge_count) == (1, 1)
    assert record.max_subturn_q == 4
    assert record.canonical_reflex_excess_over_pi == ExactRatioV1(1, 2)
    # Веер вычислительной геометрии держал бы подшаг больше `pi/q`; у угла ниже —
    # нет, и записи у него нет: закон не меняет ответ там, где менять нечего.
    _ideal_angular_support_data(context, above)
    _ideal_angular_support_data(context, below)
    assert context.canonical_subturn_fan[above.envelope_spec_id.value] is True
    assert context.canonical_subturn_fan[below.envelope_spec_id.value] is False
    assert record.evaluation_turn_sign is not ExactTurnSignV1.ZERO


def test_d4_the_below_corner_is_lifted_under_the_canonical_limit_law(compiled):
    snapshot, compilation = compiled("building_patch114", 4)
    context, above, below = _above_and_below(snapshot, compilation)
    # Угол выше 90 лифтовал и прежний закон (строгий), угол ниже — нет: это и
    # был расщеплённый счёт. Теперь оба на канонической границе.
    assert _lift(above).lift_law is STRICT
    assert _lift(below).lift_law is CANONICAL_LIFT
    assert (_lift(below).source_hidden_edge_count, _lift(below).effective_hidden_edge_count) == (2, 3)
    records = {
        item.envelope_spec_id: item
        for item in compilation.evaluation_binding_noise_records
    }
    assert set(records) == {below.envelope_spec_id}
    record = records[below.envelope_spec_id]
    assert record.effect is LIFT
    assert (record.source_hidden_edge_count, record.effective_hidden_edge_count) == (2, 3)
    assert record.max_subturn_q == 6
    # Лифт и запись цитируют ОДНИ И ТЕ ЖЕ точные знак и cos^2 шума.
    assert record.evaluation_turn_sign is _lift(below).evaluation_turn_sign
    assert record.evaluation_turn_cosine_squared == _lift(below).evaluation_turn_cosine_squared
    assert _lift(below).proven_predicates == EVALUATION_SUBTURN_LIFT_PREDICATES[CANONICAL_LIFT]


def test_the_exact_corner_keeps_its_old_law_and_gets_no_record(compiled):
    """Нулевой шум — прежний закон предела и ни одной новой записи."""

    for density, law in ((2, None), (4, EXACT_LIMIT)):
        _, compilation = compiled("mesh2_patch2", density, EXACT_FIXTURE)
        assert compilation.evaluation_binding_noise_records == frozenset()
        for spec in _specs(compilation):
            lift = _lift(spec)
            assert (None if lift is None else lift.lift_law) is law


def _independent_lateral_bound(context, spec) -> Fraction:
    """Граница боковых смещений: пересчёт без модуля закона (только Fraction)."""

    binding = context.compilation.evaluation_geometry_binding
    relation = next(
        item
        for item in context.snapshot.corner_relations
        if item.corner_relation_id == spec.source_relation_id
    )
    sector = next(
        item
        for item in context.snapshot.angular_owner_sectors
        if item.owner_sector_id == spec.owner_sector_id
    )
    incoming = context.directed_chain_vertices(
        context.uses_by_id[sector.ordered_incident_chain_use_ids[0]]
    )
    outgoing = context.directed_chain_vertices(
        context.uses_by_id[sector.ordered_incident_chain_use_ids[-1]]
    )
    ids = (incoming[-2], relation.source_vertex_id, outgoing[1])

    def point(record):
        return tuple(
            Fraction(value.numerator, value.denominator)
            for value in (record.domain_coordinate.x, record.domain_coordinate.y)
        )

    source = {
        item.source_vertex_id: point(item)
        for item in context.frame.exact_source_vertex_coordinates
    }
    bound = {item.source_vertex_id: point(item) for item in binding.source_vertex_coordinates}
    matrix = context.frame.exact_gram_matrix
    g00, g01, g11 = (
        Fraction(matrix.m00.numerator, matrix.m00.denominator),
        Fraction(matrix.m01.numerator, matrix.m01.denominator),
        Fraction(matrix.m11.numerator, matrix.m11.denominator),
    )
    determinant = g00 * g11 - g01 * g01
    worst = Fraction(0)
    for start, end in zip(ids, ids[1:]):
        edge = (source[end][0] - source[start][0], source[end][1] - source[start][1])
        shift = tuple(
            (bound[end][axis] - source[end][axis]) - (bound[start][axis] - source[start][axis])
            for axis in (0, 1)
        )
        cross = edge[0] * shift[1] - edge[1] * shift[0]
        length = g00 * edge[0] ** 2 + 2 * g01 * edge[0] * edge[1] + g11 * edge[1] ** 2
        worst = max(worst, determinant * cross * cross / length)
    return worst


@pytest.mark.parametrize("density", (2, 4))
def test_the_record_carries_the_exact_noise_witness_and_the_exact_offset_bound(
    compiled, density
):
    snapshot, compilation = compiled("building_patch114", density)
    context, _ = _context(snapshot, compilation)
    (record,) = compilation.evaluation_binding_noise_records
    spec = next(
        item for item in _specs(compilation) if item.envelope_spec_id == record.envelope_spec_id
    )
    *_, ideal = _ideal_angular_support_data(context, spec)
    sign, cosine_squared = _exact_turn_witness(context.metric, ideal)
    assert (record.evaluation_turn_sign, record.evaluation_turn_cosine_squared) == (
        sign,
        cosine_squared,
    )
    # Шум настоящий, а не нулевой: знак не ZERO и cos^2 строго положителен.
    assert sign is not ExactTurnSignV1.ZERO and cosine_squared.numerator > 0
    stored = record.binding_lateral_offset_gram_squared_bound
    assert Fraction(stored.numerator, stored.denominator) == _independent_lateral_bound(
        context, spec
    )


# --------------------------------------------------------------------------
# 3. Подделки: проверяющий пересчитывает, а не доверяет записи
# --------------------------------------------------------------------------


def _forged(compilation, **changes):
    return replace(compilation, **changes)


def _build(snapshot, compilation):
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    return GeometryContext.build(compilation, frame)


def _refused(snapshot, compilation):
    with pytest.raises(ReferenceGeometryError) as error:
        context = _build(snapshot, compilation)
        seal_angular_support_cache(context)
    return error.value


def _record_changes(record):
    return (
        {"evaluation_turn_sign": ExactTurnSignV1.ZERO},
        {"evaluation_turn_cosine_squared": ExactRatioV1(1, 4)},
        {
            "binding_lateral_offset_gram_squared_bound": ExactRationalV1(
                1, 10**30
            )
        },
        {"edge_direction_sine_squared_bound": ExactRationalV1(1, 10**30)},
        {"direction_sine_bound": ExactRationalV1(1, 10**6)},
        {"canonical_reflex_excess_over_pi": ExactRatioV1(1, 3)},
        {"max_subturn_q": 3},
        {"source_hidden_edge_count": 7},
        {"proven_predicates": frozenset({"EVERYTHING_IS_FINE"})},
        {"selection_certificate_id": SelectionCertificateId("not-a-selection")},
    )


@pytest.mark.parametrize("density", (2, 4))
def test_every_field_of_a_noise_record_is_recomputed_not_trusted(compiled, density):
    snapshot, compilation = compiled("building_patch114", density)
    (record,) = compilation.evaluation_binding_noise_records
    _build(snapshot, compilation)
    for changes in _record_changes(record):
        forged = _forged(
            compilation,
            evaluation_binding_noise_records=frozenset({replace(record, **changes)}),
        )
        assert _refused(snapshot, forged) is not None, changes


def test_stripping_the_record_of_a_canonical_fan_is_a_named_refusal(compiled):
    snapshot, compilation = compiled("building_patch114", 2)
    error = _refused(
        snapshot, _forged(compilation, evaluation_binding_noise_records=frozenset())
    )
    assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    assert "without its recorded evaluation binding noise" in str(error)


def test_stripping_the_record_of_a_canonical_limit_lift_is_a_named_refusal(compiled):
    snapshot, compilation = compiled("building_patch114", 4)
    error = _refused(
        snapshot, _forged(compilation, evaluation_binding_noise_records=frozenset())
    )
    assert error.outcome is ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID
    assert "no recorded binding noise" in str(error)


def test_a_fan_record_on_a_fan_that_already_satisfies_the_guarantee_is_refused(compiled):
    """Запись о ВЫЧИСЛИТЕЛЬНОМ шуме на угле, чей веер и так держит шаг, — подделка."""

    snapshot, compilation = compiled("building_patch114", 2)
    (record,) = compilation.evaluation_binding_noise_records
    feasible = next(
        spec
        for spec in _specs(compilation)
        if spec.envelope_spec_id != record.envelope_spec_id
    )
    planted = replace(record, envelope_spec_id=feasible.envelope_spec_id)
    error = _refused(
        snapshot,
        _forged(
            compilation,
            evaluation_binding_noise_records=frozenset({record, planted}),
        ),
    )
    assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    assert "already satisfies" in str(error)


def test_a_lift_record_without_a_canonical_limit_lift_is_refused(compiled):
    snapshot, compilation = compiled("building_patch114", 2)
    (record,) = compilation.evaluation_binding_noise_records
    below = next(
        spec for spec in _specs(compilation) if spec.envelope_spec_id != record.envelope_spec_id
    )
    stray = replace(
        record,
        envelope_spec_id=below.envelope_spec_id,
        effect=LIFT,
        proven_predicates=noise.NOISE_PREDICATES[LIFT],
    )
    error = _refused(
        snapshot,
        _forged(
            compilation,
            evaluation_binding_noise_records=frozenset({record, stray}),
        ),
    )
    assert error.outcome is ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID
    assert "no canonical-limit lift" in str(error)


def test_a_count_that_the_canonical_fan_must_lift_cannot_stay_at_the_old_count(compiled):
    """ОБРАТНАЯ сторона закона: прежний счёт 2 на d4 — решение знака шума.

    Угол ниже 90 прежний закон оставлял на двух скрытых лучах. Возврат к такой
    спеке (без лифта и без записи) проверяющий отказывает по самому веерам, а не
    по отсутствию записи.
    """

    from cftuv_envelope.reference.angular import _verify_canonical_count_law
    from cftuv_envelope.reference.compile import _density_spec_with_hidden_count

    snapshot, compilation = compiled("building_patch114", 4)
    context, _ = _context(snapshot, compilation)
    (record,) = compilation.evaluation_binding_noise_records
    lifted = next(
        spec for spec in _specs(compilation) if spec.envelope_spec_id == record.envelope_spec_id
    )
    selection = next(iter(compilation.profile_selection_certificates))
    _verify_canonical_count_law(context, lifted)
    reverted = _density_spec_with_hidden_count(lifted, 2)
    assert noise.canonical_count_law_error(context, reverted, selection) == (
        "canonical exact-limit count is not lifted: the sign of the "
        "evaluation binding noise decided it"
    )
    with pytest.raises(ReferenceGeometryError) as error:
        _verify_canonical_count_law(context, reverted)
    assert error.value.outcome is ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID
    # Счёт на один выше проходит: проверка требует лифта, а не конкретной записи.
    assert noise.canonical_count_law_error(context, lifted, selection) is None


@pytest.mark.parametrize(
    "changes",
    (
        {"lift_law": EXACT_LIMIT, "proven_predicates": EVALUATION_SUBTURN_LIFT_PREDICATES[EXACT_LIMIT]},
        {"lift_law": STRICT, "proven_predicates": EVALUATION_SUBTURN_LIFT_PREDICATES[STRICT]},
        {"evaluation_turn_sign": ExactTurnSignV1.ZERO},
        {"evaluation_turn_cosine_squared": ExactRatioV1(1, 4)},
        {"minimality_predecessor_hidden_edge_count": 1},
        {"proven_predicates": EVALUATION_SUBTURN_LIFT_PREDICATES[STRICT]},
    ),
)
def test_the_canonical_limit_lift_tampering_is_refused(compiled, changes):
    snapshot, compilation = compiled("building_patch114", 4)
    (record,) = compilation.evaluation_binding_noise_records
    spec = next(
        item for item in _specs(compilation) if item.envelope_spec_id == record.envelope_spec_id
    )
    forged_spec = replace(
        spec, evaluation_subturn_count_lift=replace(_lift(spec), **changes)
    )
    forged = _forged(
        compilation,
        envelope_specs=frozenset(
            forged_spec if item == spec else item for item in compilation.envelope_specs
        ),
    )
    assert _refused(snapshot, forged) is not None


# --------------------------------------------------------------------------
# 4. Честный околопрямой угол закона не касается
# --------------------------------------------------------------------------


@pytest.mark.parametrize("density", (2, 4))
def test_an_honest_near_right_angle_is_untouched(compiled, density):
    """90.56 градуса — не канон (вне допуска намерения): закон молчит, байты прежние."""

    snapshot, compilation = compiled("building004_patch0", density)
    context, _ = _context(snapshot, compilation)
    (spec,) = _specs(compilation)
    selection = next(iter(compilation.profile_selection_certificates))
    assert noise.canonical_noise_fact(context, spec, selection) is None
    assert compilation.evaluation_binding_noise_records == frozenset()
    assert compilation.canonical_angle_restorations == frozenset()
    assert _lift(spec) is None
    assert spec.resolved_hidden_edge_count == selection.resolved_hidden_edge_count == {2: 2, 4: 3}[density]


@pytest.mark.parametrize("density", (2, 4))
def test_a_fan_record_planted_on_an_honest_corner_is_refused(compiled, density):
    snapshot, compilation = compiled("building004_patch0", density)
    (spec,) = _specs(compilation)
    other_snapshot, other_compilation = compiled("building_patch114", 2)
    (donor,) = other_compilation.evaluation_binding_noise_records
    selection = next(iter(compilation.profile_selection_certificates))
    planted = replace(
        donor,
        envelope_spec_id=spec.envelope_spec_id,
        selection_certificate_id=selection.certificate_id,
        max_subturn_q=huber_density_value_contract(selection.max_subturn_value_id)[0],
    )
    error = _refused(
        snapshot,
        _forged(compilation, evaluation_binding_noise_records=frozenset({planted})),
    )
    assert error.outcome in (
        ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID,
        ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
    )
    assert other_snapshot is not None


@pytest.mark.parametrize("density", (3,))
def test_a_loose_threshold_is_not_governed_by_the_law(compiled, density):
    """d3 (q=5): `u*q = 5/2 != H+1` — запас положителен, закон ничего не пишет."""

    snapshot, compilation = compiled("building_patch114", density)
    _context(snapshot, compilation)
    assert compilation.evaluation_binding_noise_records == frozenset()
    assert {spec.resolved_hidden_edge_count for spec in _specs(compilation)} == {2}


# --------------------------------------------------------------------------
# 5. Решение по счёту не зависит от знака шума (синтетический шум на реальной карте)
# --------------------------------------------------------------------------


def _noisy_ideal(context, spec, count, tilt):
    """Веер счёта `count` для исходящей опоры, повёрнутой на шум `tilt` (точный)."""

    relation, sector = noise._corner_ids(context, spec)
    incoming, _ = _incident_normal(
        context, sector.ordered_incident_chain_use_ids[0], relation.source_vertex_id
    )
    outgoing, _ = _incident_normal(
        context, sector.ordered_incident_chain_use_ids[-1], relation.source_vertex_id
    )
    ix, iy = incoming.expressions()
    ox, oy = outgoing.expressions()
    tilted = ExactPlanarVector.from_values(ox + tilt * ix, oy + tilt * iy)
    return _interpolated_normals(
        context.metric,
        incoming,
        tilted,
        count,
        sector.turn_orientation,
        huber_density=True,
    )


@pytest.mark.parametrize(("density", "count"), ((2, 1), (4, 2)))
def test_synthetic_lattice_noise_up_down_zero_decides_one_count(compiled, density, count):
    """Синтетический шум ±tilt и нуль на настоящей косой карте: решение одно.

    Прежнее решение (`density_count_is_feasible` на вычислительном веере) на
    d2 зависит от знака, и это измеряется здесь же; решение закона — нет.
    """

    snapshot, compilation = compiled("mesh2_patch2", density, EXACT_FIXTURE)
    context, _ = _context(snapshot, compilation)
    spec = _specs(compilation)[0]
    selection = next(iter(compilation.profile_selection_certificates))
    q = huber_density_value_contract(selection.max_subturn_value_id)[0]
    assert canonical_count_is_tight(Fraction(1, 2), count, q)
    verdicts = {}
    plain = {}
    for label, tilt in (("up", sp.Rational(1, 5000)), ("zero", sp.Integer(0)), ("down", sp.Rational(-1, 5000))):
        ideal = _noisy_ideal(context, spec, count, tilt)
        verdicts[label] = noise.evaluation_count_is_feasible(
            context, spec, selection, count, ideal, q
        )
        plain[label] = _density_ideal_is_subturn_feasible(context.metric, ideal, q)
    assert len(set(verdicts.values())) == 1, verdicts
    assert verdicts["zero"] is (density == 2)
    # До закона знак шума решал: один из двух знаков делал веер неосуществимым
    # на шуме, а нуль и другой знак — нет (подшаг ровно `pi/q` либо чуть меньше).
    assert plain["zero"] is True
    assert {plain["up"], plain["down"]} == {True, False}, plain


# --------------------------------------------------------------------------
# 6. Арифметика закона — точная и без допуска
# --------------------------------------------------------------------------


def _interval(lower: Fraction, upper: Fraction):
    class Interval:
        pass

    item = Interval()
    item.lower, item.upper = lower, upper
    item.lower_kind = item.upper_kind = IntervalEndpointKind.CLOSED
    return item


@pytest.mark.parametrize(
    ("lower", "upper", "found"),
    (
        (Fraction(1, 2), Fraction(1, 2), True),
        # Внутри допуска восстановления (0.1 градуса, `u` до 5.55e-4): восстанавливается и тоже канон.
        (Fraction(1, 2) + Fraction(1, 10**8), Fraction(1, 2) + Fraction(1, 10**8), True),
        (Fraction(1, 2) - Fraction(1, 10**8), Fraction(1, 2) - Fraction(1, 10**8), True),
        (Fraction(1, 2) + Fraction(1, 10**4), Fraction(1, 2) + Fraction(1, 10**4), True),
        (Fraction(1, 2) - Fraction(5, 10**4), Fraction(1, 2) - Fraction(5, 10**4), True),
        # Вне допуска — честное число, закона нет.
        (Fraction(1, 2) + Fraction(6, 10**4), Fraction(1, 2) + Fraction(6, 10**4), False),
        (Fraction(1, 2) + Fraction(1, 10**3), Fraction(1, 2) + Fraction(1, 10**3), False),
        (Fraction(3, 5), Fraction(3, 5), False),
    ),
)
def test_the_canonical_fact_is_exact_or_restored_and_nothing_else(lower, upper, found):
    fact = exact_canonical_selector_fact(_interval(lower, upper))
    assert (fact is not None) is found
    if found:
        assert fact[1] == Fraction(1, 2)


@pytest.mark.parametrize(
    ("canonical", "count", "q", "tight"),
    (
        (Fraction(1, 2), 1, 4, True),
        (Fraction(1, 2), 2, 6, True),
        (Fraction(1, 2), 1, 2, False),
        (Fraction(1, 2), 1, 3, False),
        (Fraction(1, 2), 2, 5, False),
        (Fraction(1, 2), 3, 6, False),
        (Fraction(1, 2), 2, 4, False),
    ),
)
def test_tightness_is_an_integer_equality(canonical, count, q, tight):
    assert canonical_count_is_tight(canonical, count, q) is tight


def test_the_edge_noise_ignores_a_shift_along_the_chain_and_measures_the_direction_exactly():
    """Продольный сдвиг направления не меняет; sin^2 поворота — точное рациональное число."""

    gram = (Fraction(1), Fraction(0), Fraction(1))
    source = {"a": (Fraction(0), Fraction(0)), "b": (Fraction(2), Fraction(0))}
    along = {"a": (Fraction(0), Fraction(0)), "b": (Fraction(5, 2), Fraction(0))}
    across = {"a": (Fraction(0), Fraction(0)), "b": (Fraction(2), Fraction(1, 10))}
    assert noise._edge_noise(gram, source, along, "a", "b")[:2] == (0, 0)
    lateral, sine_squared, moved = noise._edge_noise(gram, source, across, "a", "b")
    assert lateral == Fraction(1, 100)
    # sin^2 = cross^2 / (|edge|^2 * |edge'|^2) = (1/5)^2 / (4 * 401/100) = 1/401.
    assert sine_squared == Fraction(1, 401)
    assert moved == (Fraction(2), Fraction(1, 10))
    skew = (Fraction(2), Fraction(1, 2), Fraction(3))
    # В косой карте: det(G) * cross^2 / |edge|^2, точно, в рациональных числах.
    assert noise._edge_noise(skew, source, across, "a", "b")[0] == (
        (2 * 3 - Fraction(1, 4)) * (2 * Fraction(1, 10)) ** 2 / (2 * 4)
    )
    # Вырожденное ребро — не деление на нуль, а отказ.
    assert noise._edge_noise(gram, source, {"a": source["a"], "b": source["a"]}, "a", "b") is None


def test_the_lift_predicate_table_names_the_canonical_limit_law():
    assert CANONICAL_LIFT in EVALUATION_SUBTURN_LIFT_PREDICATES
    assert "SELECTOR_INTERVAL_IS_EXACTLY_CANONICAL" in EVALUATION_SUBTURN_LIFT_PREDICATES[CANONICAL_LIFT]
    assert set(noise.NOISE_PREDICATES) == set(EvaluationBindingNoiseEffectV1)


# --------------------------------------------------------------------------
# 7. Угловая граница применимости: боковой сдвиг в ячейку угол не ограничивает
# --------------------------------------------------------------------------


def _bound_noise(**changes):
    values = dict(
        lateral_squared=Fraction(1, 10**6),
        cell_bound=Fraction(1, 10**5),
        edge_sine_squared=Fraction(1, 10**9),
        turn_cosine_squared=Fraction(1, 10**9),
    )
    values.update(changes)
    return noise._BindingNoise(**values)


_RIGHT = (object(), Fraction(1, 2))
REFUSALS = EvaluationBindingNoiseRefusalV1


def test_a_short_edge_passes_the_cell_gate_and_is_refused_by_the_angle():
    """Ребро короче сдвига: боковой сдвиг в ячейке, а поворот направления — нет.

    Раньше это проходило (граница ограничивала только сдвиг). Теперь точное
    угловое условие отказывает по ИМЕНИ, и счёт решает прежний закон.
    """

    short_edge = _bound_noise(edge_sine_squared=Fraction(99, 10**4))
    assert short_edge.lateral_squared <= short_edge.cell_bound
    assert noise._noise_refusal(short_edge, _RIGHT) is (
        REFUSALS.EDGE_DIRECTION_NOISE_EXCEEDS_DECLARED_BOUND
    )
    assert noise._noise_refusal(_bound_noise(), _RIGHT) is None


def test_every_boundary_of_the_law_refuses_by_its_own_name():
    bound_squared = noise.NOISE_DIRECTION_SINE_BOUND**2
    assert noise._noise_refusal(
        _bound_noise(lateral_squared=Fraction(1, 10**4)), _RIGHT
    ) is REFUSALS.LATERAL_SHIFT_EXCEEDS_ONE_LATTICE_CELL
    assert noise._noise_refusal(
        _bound_noise(turn_cosine_squared=bound_squared * 2), _RIGHT
    ) is REFUSALS.TURN_NOISE_EXCEEDS_DECLARED_BOUND
    # Граница включительна и точна: равенство проходит, превышение на долю — нет.
    assert noise._noise_refusal(
        _bound_noise(edge_sine_squared=bound_squared, turn_cosine_squared=bound_squared),
        _RIGHT,
    ) is None
    assert noise._noise_refusal(
        _bound_noise(edge_sine_squared=bound_squared + Fraction(1, 10**30)), _RIGHT
    ) is REFUSALS.EDGE_DIRECTION_NOISE_EXCEEDS_DECLARED_BOUND
    # Отношение без углового предела: закон не определён.
    assert noise._noise_refusal(_bound_noise(), (object(), Fraction(1, 3))) is (
        REFUSALS.CANONICAL_RELATION_HAS_NO_ANGULAR_BOUND
    )


def _measured_ceiling(compilation, snapshot) -> float:
    """Наибольший синус из измеренных законом на фикстуре (по рёбрам и повороту)."""

    context, _ = _context(snapshot, compilation)
    selection = next(iter(compilation.profile_selection_certificates))
    worst = max(
        max(fact.edge_direction_sine_squared_bound, fact.turn_cosine_squared)
        for fact in (
            noise.canonical_noise_fact(context, spec, selection)
            for spec in _specs(compilation)
        )
    )
    return float(worst) ** 0.5


def test_the_declared_bound_decides_exactly_where_the_law_applies(compiled, monkeypatch):
    """Граница чуть выше измеренного шума — закон работает, чуть ниже — отказ по имени."""

    import math

    snapshot, compilation = compiled("building_patch114", 2)
    ceiling = _measured_ceiling(compilation, snapshot)
    above = Fraction(1, math.floor(1 / ceiling))
    below = Fraction(1, math.ceil(1 / ceiling) + 1)
    assert below < Fraction(ceiling) < above

    loaded = _load(FIXTURE, "building_patch114", 2)
    monkeypatch.setattr(noise, "NOISE_DIRECTION_SINE_BOUND", above)
    applied = compile_reference_envelopes(*loaded)
    assert len(applied.compilation.evaluation_binding_noise_records) == 1
    assert applied.compilation.diagnostics == ()

    monkeypatch.setattr(noise, "NOISE_DIRECTION_SINE_BOUND", below)
    refused = compile_reference_envelopes(*loaded)
    assert refused.outcome is ReferenceOutcome.EXACT, refused.diagnostics
    compilation_refused = refused.compilation
    # Закон не применён: прежний закон на вычислительной геометрии, знак шума решает.
    assert compilation_refused.evaluation_binding_noise_records == frozenset()
    assert sorted(
        spec.resolved_hidden_edge_count for spec in _specs(compilation_refused)
    ) == [1, 2]


def test_a_refusal_of_the_law_is_named_in_the_diagnostics_and_counted(compiled, monkeypatch):
    """Молчаливого отката к знаку шума нет: диагностика ядра и счётчик конвейера."""

    import math

    from cftuv_envelope.wavefront import prepare_conveyor

    snapshot, compilation = compiled("building_patch114", 2)
    ceiling = _measured_ceiling(compilation, snapshot)
    snapshot_loaded, request = _load(FIXTURE, "building_patch114", 2)

    default = prepare_conveyor(snapshot_loaded, request)
    assert "CONVEYOR_BINDING_NOISE_LAW_REFUSED" not in dict(default.counters)

    monkeypatch.setattr(
        noise, "NOISE_DIRECTION_SINE_BOUND", Fraction(1, math.ceil(1 / ceiling) + 1)
    )
    refused = compile_reference_envelopes(snapshot_loaded, request)
    # Шум вне границ молчит ДВАЖДЫ и по двум решениям: счёт на тугом пороге
    # (закон шума) и лучи привязанного веера (закон лучей, та же причина). Закон лучей
    # называет только угол, у которого ЕСТЬ строка таблицы (поднятый d2 `H = 2`); у тугого
    # d2 `H = 1` строки нет намеренно, и шум закону лучей нечем называть.
    assert len(refused.compilation.diagnostics) == 3
    diagnostics = [
        item
        for item in refused.compilation.diagnostics
        if item.outcome is ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED
    ]
    rays = [
        item
        for item in refused.compilation.diagnostics
        if item.outcome is ReferenceOutcome.CANONICAL_FAN_RAYS_LAW_NOT_APPLIED
    ]
    assert (len(diagnostics), len(rays)) == (2, 1)
    assert all(
        item.message.startswith("BINDING_NOISE_OUTSIDE_THE_DECLARED_BOUNDS")
        for item in rays
    )
    for item in diagnostics:
        assert item.outcome is ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED
        assert item.message.startswith(
            EvaluationBindingNoiseRefusalV1.EDGE_DIRECTION_NOISE_EXCEEDS_DECLARED_BOUND.value
        ) or item.message.startswith(
            EvaluationBindingNoiseRefusalV1.TURN_NOISE_EXCEEDS_DECLARED_BOUND.value
        )
        assert item.envelope_spec_id is not None
    prepared = prepare_conveyor(snapshot_loaded, request)
    assert dict(prepared.counters)["CONVEYOR_BINDING_NOISE_LAW_REFUSED"] == 2


def test_a_record_is_judged_under_the_declared_bound_not_under_the_one_it_was_written_with(
    compiled, monkeypatch
):
    import math

    snapshot, compilation = compiled("building_patch114", 2)
    # Запись написана под 1/1000; граница затянута ниже измеренного шума, и закон
    # к углу больше не применим — проверяющий пересчитывает и отказывает.
    ceiling = _measured_ceiling(compilation, snapshot)
    monkeypatch.setattr(
        noise, "NOISE_DIRECTION_SINE_BOUND", Fraction(1, math.ceil(1 / ceiling) + 1)
    )
    error = _refused(snapshot, compilation)
    assert error.outcome is ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID
    assert "not in force" in str(error)
    # И обратно: запись под другой объявленной границей не следует из геометрии.
    monkeypatch.setattr(noise, "NOISE_DIRECTION_SINE_BOUND", Fraction(1, 500))
    assert "does not follow from the geometry" in str(_refused(snapshot, compilation))


def test_missing_source_coordinates_and_corner_vertices_are_named_refusals(compiled):
    """Без исходных координат закон не угадывает шум и не молчит: причина названа."""

    from types import SimpleNamespace

    snapshot, compilation = compiled("building_patch114", 2)
    context, _ = _context(snapshot, compilation)
    spec = _specs(compilation)[0]
    selection = next(iter(compilation.profile_selection_certificates))

    bare = replace(
        context,
        frame=SimpleNamespace(exact_gram_matrix=context.frame.exact_gram_matrix),
        evaluation_noise_cache={},
    )
    outcome = noise.canonical_noise_applicability(bare, spec, selection)
    assert outcome.fact is None and outcome.canonical is not None
    assert outcome.refusal is REFUSALS.SOURCE_COORDINATES_UNAVAILABLE

    without_binding = replace(
        context,
        compilation=replace(compilation, evaluation_geometry_binding=None),
        evaluation_noise_cache={},
    )
    outcome = noise.canonical_noise_applicability(without_binding, spec, selection)
    # Привязки нет — шума нет: закону нечем молчать, и отказа тоже нет.
    assert outcome.fact is None and outcome.refusal is None

    other = replace(context, evaluation_noise_cache={})
    original = noise._corner_vertices
    try:
        noise._corner_vertices = lambda *args: None
        outcome = noise.canonical_noise_applicability(other, spec, selection)
    finally:
        noise._corner_vertices = original
    assert outcome.refusal is REFUSALS.CORNER_VERTICES_UNAVAILABLE


# --------------------------------------------------------------------------
# 8. Каждый сектор канонического веера
# --------------------------------------------------------------------------


@pytest.mark.parametrize(("density", "count"), ((2, 1), (4, 2)))
def test_every_sector_of_the_canonical_fan_is_checked_and_a_wide_one_is_caught(
    compiled, density, count
):
    snapshot, compilation = compiled("building_patch114", density)
    context, _ = _context(snapshot, compilation)
    spec = _specs(compilation)[0]
    q = 2 * (count + 1)
    assert noise.canonical_sectors_hold(
        context, noise.canonical_ideal(context, spec, count, Fraction(1, 2)), q
    )
    # Тот же веер на «канон» 3/5: подшаг 3/10 pi больше pi/4 — сектор шире порога.
    wide = noise.canonical_ideal(context, spec, count, Fraction(3, 5))
    assert not noise.canonical_sectors_hold(context, wide, q)


@pytest.mark.parametrize("density", (2, 4))
def test_the_last_sector_exceeds_pi_over_q_by_exactly_the_recorded_turn_noise(
    compiled, density
):
    """Последний сектор канонического веера = `pi/q` + отклонение поворота — числом.

    Первые `H` секторов — точные повороты; последний ведёт к опоре
    вычислительной геометрии, и его превышение над `pi/q` равно отклонению
    поворота от прямого угла, чей синус в квадрате — записанный `cos^2`.
    """

    import math

    snapshot, compilation = compiled("building_patch114", density)
    context, _ = _context(snapshot, compilation)
    selection = next(iter(compilation.profile_selection_certificates))
    q = huber_density_value_contract(selection.max_subturn_value_id)[0]
    metric = context.metric
    gram = tuple(tuple(float(item) for item in row) for row in metric.gram)

    def dot(left, right):
        lx, ly = (float(sp.N(item, 30)) for item in metric.density_expressions(left))
        rx, ry = (float(sp.N(item, 30)) for item in metric.density_expressions(right))
        return (
            lx * (gram[0][0] * rx + gram[0][1] * ry)
            + ly * (gram[1][0] * rx + gram[1][1] * ry)
        )

    for spec in _specs(compilation):
        fact = noise.canonical_noise_fact(context, spec, selection)
        count = selection.resolved_hidden_edge_count
        ideal = noise.canonical_ideal(context, spec, count, Fraction(1, 2))
        cosine = dot(ideal[-2], ideal[-1]) / math.sqrt(
            dot(ideal[-2], ideal[-2]) * dot(ideal[-1], ideal[-1])
        )
        last = math.acos(max(-1.0, min(1.0, cosine)))
        deviation = math.asin(math.sqrt(float(fact.turn_cosine_squared)))
        assert abs(abs(last - math.pi / q) - deviation) < 1e-9, (last, deviation)
