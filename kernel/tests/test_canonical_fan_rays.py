"""CANONICAL-FAN-RAYS: одинаковые канонические углы получают один веер.

Полевой дефект (`artifacts/fan_consistency/fan_steps.py`, замер 2026-10-03): на
d4 лифт счёта канонического прямого угла давал равноугольный идеал `pi/8`, а
лучи искал адаптивный атлас на шумной вычислительной геометрии. Сорок восемь
конгруэнтных углов стены `2` получали десять разных наборов шагов. Закон
`CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1` ставит лучи по таблице рациональных
поворотов входящей опоры, а независимый проверяющий пересчитывает веер по
сырой геометрии и сверяет лучи на точное равенство направления.

Данные — выгрузки живой сцены владельца: `binding_noise_canonical_v1`
(`building_patch114` несёт углы обоих знаков шума привязки) и
`density4_exact_limit_v1` (`mesh2_patch2` — точный прямой угол, нулевой шум).
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp
from mpmath import iv

import cftuv_envelope as kernel
from cftuv_envelope import AngularEnvelopeSpec, EvaluationGeometrySubturnCountLiftLawV1
from cftuv_envelope import _density_policy
from cftuv_envelope._density_policy import (
    CANONICAL_FAN_RAYS_PREDICATES,
    CANONICAL_ROTATION_TABLE,
    canonical_rotation_rays,
)
from cftuv_envelope.adaptive_density_validation import adaptive_density_structure_errors
from cftuv_envelope.codec import _decode_as, to_canonical_data
from cftuv_envelope.contracts.envelopes import (
    AdaptiveBoundHiddenSupportDirectionLawV2,
    AdaptiveDensityAngularEnvelopeSpecV2,
    AdaptiveMinimalRationalFanAuthorityV2,
    CanonicalFanRaysLawV1,
    CanonicalRationalRotationFanAuthorityV1,
)
from cftuv_envelope.reference import ReferenceOutcome, compile_reference_envelopes
from cftuv_envelope.reference import direction_binding
from cftuv_envelope.reference.adaptive_density_fan import _covectors, _dual_dot
from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.reference.angular import (
    _interpolated_normals,
    angular_support_data,
    seal_angular_support_cache,
)
from cftuv_envelope.reference.canonical_fan_rays import (
    _primitive_covector,
    canonical_fan_rays_decision,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.contracts import CanonicalFanRaysRefusalV1
from cftuv_envelope.reference.validation import validate_reference_geometry_payload
from cftuv_envelope.wavefront import prepare_conveyor

KERNEL = Path(__file__).resolve().parents[1]
NOISE_FIXTURE = KERNEL / "fixtures" / "binding_noise_canonical_v1"
EXACT_FIXTURE = KERNEL / "fixtures" / "density4_exact_limit_v1"

#: Угловой веер канонического прямого угла при `q = 6`, градусы: пары таблицы
#: `(12, 5)`, `(1, 1)`, `(5, 12)` дают лучи на 22.62, 45 и 67.38 градуса.
TABLE_ROW = ((12, 5), (1, 1), (5, 12))
FIRST_SECTOR = math.degrees(math.atan2(5, 12))
#: Объявленный синус шума привязки закона `1/1000` в градусах: шум последнего сектора.
NOISE_BOUND_DEGREES = math.degrees(math.asin(1 / 1000))
MIDDLE_SECTOR = math.degrees(math.atan2(1, 1) - math.atan2(5, 12))

STRICT = EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1
EXACT_LIMIT = (
    EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1
)
CANONICAL_LIMIT = (
    EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1
)

#: (папка, имя) каждого канонического слепка d4.
CANONICAL_CASES = (
    (NOISE_FIXTURE, "building_patch114"),
    (NOISE_FIXTURE, "building_patch4"),
    (EXACT_FIXTURE, "mesh2_patch2"),
    (EXACT_FIXTURE, "building_patch3"),
    (EXACT_FIXTURE, "building_patch20"),
)


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

    def get(folder: Path, name: str, density: int):
        key = (folder, name, density)
        if key not in cache:
            snapshot, request = _load(folder, name, density)
            result = compile_reference_envelopes(snapshot, request)
            assert result.outcome is ReferenceOutcome.EXACT, (name, density)
            cache[key] = (snapshot, result.compilation)
        return cache[key]

    return get


def _specs(compilation):
    return sorted(
        (item for item in compilation.envelope_specs if isinstance(item, AngularEnvelopeSpec)),
        key=lambda item: item.envelope_spec_id.value,
    )


def _lifted(compilation):
    return [
        spec
        for spec in _specs(compilation)
        if getattr(spec, "evaluation_subturn_count_lift", None) is not None
    ]


def _context(snapshot, compilation):
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    return GeometryContext.build(compilation, frame)


def _refused(snapshot, compilation):
    with pytest.raises(ReferenceGeometryError) as error:
        context = _context(snapshot, compilation)
        seal_angular_support_cache(context)
    return error.value


def _fan_degrees(context, spec) -> list[float]:
    """Углы секторов веера в метрике вычислительной геометрии (отчёт, не решение)."""

    *_, normals = angular_support_data(context, spec)
    covectors = _covectors(context.metric, normals)

    def cosine(left, right):
        return _dual_dot(context.metric, left, right) / sp.sqrt(
            _dual_dot(context.metric, left, left)
            * _dual_dot(context.metric, right, right)
        )

    return [
        math.degrees(math.acos(float(sp.N(cosine(left, right), 30))))
        for left, right in zip(covectors, covectors[1:])
    ]


def _replace_spec(compilation, spec, forged):
    return replace(
        compilation,
        envelope_specs=frozenset((compilation.envelope_specs - {spec}) | {forged}),
    )


def _forge_authority(spec, **changes):
    authority = replace(spec.direction_fan_authority, **changes)
    supports = frozenset(
        replace(
            support,
            bound_primitive_integer_vector=authority.bound_primitive_integer_vectors[
                support.ordinal - 1
            ],
            direction_fan_authority_id=authority.authority_id,
        )
        for support in spec.hidden_supports
    )
    return replace(spec, direction_fan_authority=authority, hidden_supports=supports)


# --------------------------------------------------------------------------
# 1. Таблица: условия записи проверяются точно, а не по памяти
# --------------------------------------------------------------------------


def _angle(a: int, b: int):
    return iv.atan2(iv.mpf(b), iv.mpf(a))


def test_the_rotation_table_is_exactly_what_its_header_promises():
    """Ряд — палиндром по углам, строго возрастает, и каждый сектор канона `<= pi/q`.

    Строка таблицы — новое обещание о каждом домене, где она сработает, поэтому
    для неё проверяется ВСЁ заявленное: порядок, симметрия и потолок сектора на
    ГРАНИЦЕ КАНОНА (первый сектор от входящей опоры, последний — до исходящей).
    Новая запись с другой долей `u` требует нового доказательства: тест падает
    на ней, а не пропускает молча.
    """

    assert CANONICAL_ROTATION_TABLE
    saved_precision = iv.prec
    iv.prec = 200
    try:
        for (canonical, sector_count, q), row in CANONICAL_ROTATION_TABLE.items():
            assert canonical == Fraction(1, 2), "a new angle needs its own proof"
            assert len(row) == sector_count - 1
            assert all(a > 0 and b > 0 and isinstance(a, int) for a, b in row)
            # Симметрия `atan(b/a) + atan(b'/a') = pi/2` — точно: `b b' = a a'`.
            for index, (a, b) in enumerate(row):
                mirror_a, mirror_b = row[len(row) - 1 - index]
                assert b * mirror_b == a * mirror_a, (canonical, sector_count, q)
            angles = [_angle(a, b) for a, b in row]
            ceiling = iv.pi / q
            bounds = [iv.mpf(0), *angles, iv.pi * canonical.numerator / canonical.denominator]
            for before, after in zip(bounds, bounds[1:]):
                assert (after - before).b < ceiling.a, (canonical, sector_count, q)
            assert all(
                (later - earlier).a > 0 for earlier, later in zip(angles, angles[1:])
            )
    finally:
        iv.prec = saved_precision
    assert canonical_rotation_rays(Fraction(1, 2), 4, 6) == TABLE_ROW
    assert canonical_rotation_rays(Fraction(1, 2), 3, 4) is None


# --------------------------------------------------------------------------
# 2. Один веер на всех канонических углах, при любом знаке шума
# --------------------------------------------------------------------------


@pytest.mark.parametrize("folder,name", CANONICAL_CASES)
def test_every_lifted_canonical_corner_carries_the_table_fan(compiled, folder, name):
    """Лифт любого из трёх законов, угол выше, ниже и точный: одна власть и одни лучи."""

    snapshot, compilation = compiled(folder, name, 4)
    lifted = _lifted(compilation)
    assert lifted
    context = _context(snapshot, compilation)
    seal_angular_support_cache(context)
    for spec in lifted:
        authority = spec.direction_fan_authority
        assert type(authority) is CanonicalRationalRotationFanAuthorityV1
        assert authority.ray_law is CanonicalFanRaysLawV1.CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1
        assert authority.ray_rotation_pairs == TABLE_ROW
        assert authority.proven_predicates == CANONICAL_FAN_RAYS_PREDICATES
        assert {support.direction_law for support in spec.hidden_supports} == {
            AdaptiveBoundHiddenSupportDirectionLawV2.CANONICAL_RATIONAL_ROTATION_FAN_V1
        }
        assert adaptive_density_structure_errors(spec) == ()
        steps = _fan_degrees(context, spec)
        # Первые три сектора — ТОЧНЫЕ повороты таблицы; шум привязки целиком
        # в последнем секторе и в пределах объявленного синуса `1/1000`.
        assert steps[0] == pytest.approx(FIRST_SECTOR, abs=1e-8)
        assert steps[1] == pytest.approx(MIDDLE_SECTOR, abs=1e-8)
        assert steps[2] == pytest.approx(MIDDLE_SECTOR, abs=1e-8)
        assert steps[3] == pytest.approx(FIRST_SECTOR, abs=NOISE_BOUND_DEGREES)


def test_the_noise_signs_and_the_exact_corner_give_one_ray_sequence(compiled):
    """Угол ниже 90, выше 90 и точный 90 — те же лучи; шум — только последний сектор."""

    laws = set()
    noise = {-1: 0, 0: 0, 1: 0}
    for folder, name in CANONICAL_CASES:
        snapshot, compilation = compiled(folder, name, 4)
        context = _context(snapshot, compilation)
        for spec in _lifted(compilation):
            laws.add(spec.evaluation_subturn_count_lift.lift_law)
            steps = _fan_degrees(context, spec)
            excess = sum(steps) - 90.0
            noise[(excess > 1e-7) - (excess < -1e-7)] += 1
            assert steps[0] == pytest.approx(FIRST_SECTOR, abs=1e-8)
            assert steps[1] == pytest.approx(MIDDLE_SECTOR, abs=1e-8)
    # Все три закона лифта и оба знака шума представлены ЭТИМИ данными.
    assert laws == {STRICT, EXACT_LIMIT, CANONICAL_LIMIT}
    assert noise[-1] and noise[1] and noise[0]


def test_two_corners_with_different_noise_get_the_same_primitive_rays_in_their_own_frame(
    compiled,
):
    """Угол с шумом и угол без шума: отношение лучей к входящей опоре одно и то же.

    Сравнение в кадре входящей опоры точное: проекции каждого луча на `e` и `J e`
    пропорциональны паре таблицы, поэтому `b_j / a_j` луча `j` равно паре без
    единого допуска. Здесь это читается как равенство углов лучей от входящей
    опоры с точностью до `1e-8` градуса — ниже любого шума привязки.
    """

    angles = {}
    for folder, name in ((NOISE_FIXTURE, "building_patch114"), (EXACT_FIXTURE, "mesh2_patch2")):
        snapshot, compilation = compiled(folder, name, 4)
        context = _context(snapshot, compilation)
        for spec in _lifted(compilation):
            steps = _fan_degrees(context, spec)
            angles[(name, spec.envelope_spec_id)] = [
                sum(steps[: index + 1]) for index in range(3)
            ]
    assert len(angles) >= 3
    for ray_angles in angles.values():
        assert ray_angles[0] == pytest.approx(math.degrees(math.atan2(5, 12)), abs=1e-8)
        assert ray_angles[1] == pytest.approx(45.0, abs=1e-8)
        assert ray_angles[2] == pytest.approx(math.degrees(math.atan2(12, 5)), abs=1e-8)


# --------------------------------------------------------------------------
# 2b. Зеркальные углы: обратный обход даёт зеркальные лучи ТОЧНО
# --------------------------------------------------------------------------


def _chart_metric(g00: Fraction, g01: Fraction, g11: Fraction) -> ExactPlanarMetric:
    gram = tuple(
        tuple(sp.Rational(item.numerator, item.denominator) for item in row)
        for row in ((g00, g01), (g01, g11))
    )
    inverse = sp.Matrix(gram).inv()
    return ExactPlanarMetric(
        gram, ((inverse[0, 0], inverse[0, 1]), (inverse[1, 0], inverse[1, 1])), 1
    )


#: Карты: единичная и две косые из слепков владельца (определитель Грама — полный квадрат).
MIRROR_CHARTS = {
    "identity": (Fraction(1), Fraction(0), Fraction(1)),
    "mesh2": (Fraction(1, 4), Fraction(1, 4), Fraction(1843815665, 2**28)),
    "building": (
        Fraction(3893136025, 2**26),
        Fraction(1037192085, 2**25),
        Fraction(280518433, 2**24),
    ),
}

#: Равноугольный ряд `(12 + 5i)^k` — НЕ палиндром: отрицательный контроль теста.
NON_PALINDROME_ROW = ((12, 5), (119, 120), (828, 2035))


def _primitive(vector) -> tuple[int, int]:
    """Целый примитивный вектор с положительным масштабом из рационального."""

    denominator = math.lcm(*(sp.Rational(item).q for item in vector))
    scaled = [int(item * denominator) for item in vector]
    divisor = math.gcd(*scaled)
    return scaled[0] // divisor, scaled[1] // divisor


def _mirror_rays(chart, clockwise: bool, row):
    """`(лучи угла A, лучи зеркального угла B в обратном обходе, M^{-T})`, примитивные ковекторы.

    `M` — отражение карты через ось первого базисного вектора (изометрия Грама);
    угол B — образ угла A под `M`, пройденный В ОБРАТНУЮ СТОРОНУ: входящая опора
    B — образ исходящей опоры A и наоборот.
    """

    g00, g01, g11 = chart
    metric = _chart_metric(g00, g01, g11)
    gram = sp.Matrix(metric.gram)
    mirror = sp.Matrix([[1, 2 * sp.Rational(g01.numerator, g01.denominator) / sp.Rational(g00.numerator, g00.denominator)], [0, -1]])
    assert mirror.T * gram * mirror == gram, "M is not an isometry of the chart"
    sign = -1 if clockwise else 1
    orientation = (
        TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
        if clockwise
        else TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    )
    incoming = sp.Matrix([1, 0])
    # Вектор, Грам-ортогональный `incoming`: прямой канонический угол, точно.
    outgoing = sign * sp.Matrix(
        [-sp.Rational(g01.numerator, g01.denominator), sp.Rational(g00.numerator, g00.denominator)]
    )
    assert (incoming.T * gram * outgoing)[0] == 0
    mirrored_incoming, mirrored_outgoing = mirror * outgoing, mirror * incoming

    def rays(first, second):
        ideal = _interpolated_normals(
            metric,
            ExactPlanarVector.from_values(*first),
            ExactPlanarVector.from_values(*second),
            len(row),
            orientation,
            huber_density=True,
            rational_rotation=row,
        )
        return [_primitive_covector(metric, ray) for ray in ideal[1:-1]]

    # Обратный обход отражённого угла сохраняет ориентацию: det(M) = -1 и
    # смена порядка опор — два переворота.
    original = rays(tuple(incoming), tuple(outgoing))
    reflected = rays(tuple(mirrored_incoming), tuple(mirrored_outgoing))
    return original, reflected, mirror.inv().T


@pytest.mark.parametrize("clockwise", (False, True))
@pytest.mark.parametrize("chart_name", tuple(MIRROR_CHARTS))
def test_the_mirror_corner_in_reverse_traversal_has_the_mirrored_primitive_rays(
    chart_name, clockwise
):
    """Зеркальные углы получают зеркальные веера ТОЧНО, а не «похожие».

    Угол B — отражение угла A изометрией карты `M`, пройденное в обратную
    сторону (так левый и правый угол окна видит обход патча). Луч `j` угла B —
    это образ луча `H + 1 - j` угла A: ковекторы переходят под `M^{-T}`, и
    примитивные целые векторы РАВНЫ, а не пропорциональны с допуском. Рациональная
    матрица коммутирует с изометриями решётки, а палиндромный ряд таблицы
    переставляет сектора зеркально.
    """

    original, reflected, transform = _mirror_rays(
        MIRROR_CHARTS[chart_name], clockwise, TABLE_ROW
    )
    assert len(original) == len(reflected) == len(TABLE_ROW)
    for ordinal, vector in enumerate(reflected, start=1):
        image = transform * sp.Matrix(original[len(original) - ordinal])
        assert vector == _primitive(tuple(image)), (chart_name, clockwise, ordinal)
    # Не тривиальное совпадение: лучи угла A попарно различны.
    assert len(set(original)) == len(original)


@pytest.mark.parametrize("chart_name", tuple(MIRROR_CHARTS))
def test_a_row_that_is_not_a_palindrome_breaks_the_mirror_congruence(chart_name):
    """Отрицательный контроль: несимметричный ряд (j поворотов на подшаг) зеркало ломает.

    Ради этого ряд таблицы — палиндром. Тест выше без этого контроля мог бы
    проходить на любом ряду; здесь видно, что он различает.
    """

    original, reflected, transform = _mirror_rays(
        MIRROR_CHARTS[chart_name], False, NON_PALINDROME_ROW
    )
    images = [
        _primitive(tuple(transform * sp.Matrix(original[len(original) - ordinal])))
        for ordinal in range(1, len(original) + 1)
    ]
    assert images != reflected


# --------------------------------------------------------------------------
# 3. Подделки: проверяющий пересчитывает, а не доверяет записи
# --------------------------------------------------------------------------


def _table_spec(compiled):
    snapshot, compilation = compiled(EXACT_FIXTURE, "building_patch20", 4)
    return snapshot, compilation, _lifted(compilation)[0]


def test_a_forged_ray_is_a_named_refusal(compiled):
    snapshot, compilation, spec = _table_spec(compiled)
    original = spec.direction_fan_authority.bound_primitive_integer_vectors
    for index in range(3):
        vectors = list(original)
        x, y = vectors[index]
        vectors[index] = (x + 1, y - 1) if (x + 1, y - 1) != (0, 0) else (x + 2, y)
        forged = _forge_authority(spec, bound_primitive_integer_vectors=tuple(vectors))
        error = _refused(snapshot, _replace_spec(compilation, spec, forged))
        assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
        assert "exact ordinal rotation" in str(error)
    swapped = (original[1], original[0], original[2])
    error = _refused(
        snapshot,
        _replace_spec(
            compilation,
            spec,
            _forge_authority(spec, bound_primitive_integer_vectors=swapped),
        ),
    )
    assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID


@pytest.mark.parametrize(
    "changes,fragment",
    (
        ({"ray_rotation_pairs": ((12, 5), (1, 1), (5, 13))}, "table entry"),
        ({"ray_rotation_pairs": ((5, 12), (1, 1), (12, 5))}, "table entry"),
        ({"proven_predicates": frozenset({"EVERYTHING_IS_FINE"})}, "does not follow"),
        ({"max_subturn_q": 5}, "table entry"),
        ({"hidden_edge_count": 2}, "table entry"),
    ),
)
def test_every_field_of_the_authority_is_recomputed_not_trusted(
    compiled, changes, fragment
):
    snapshot, compilation, spec = _table_spec(compiled)
    forged = _forge_authority(spec, **changes)
    error = _refused(snapshot, _replace_spec(compilation, spec, forged))
    assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    assert fragment in str(error)


def test_an_atlas_fan_on_an_angle_where_the_law_applies_is_refused(compiled, monkeypatch):
    """ОБРАТНАЯ сторона закона: канонический лифтованный угол не вправе нести атлас."""

    snapshot, request = _load(EXACT_FIXTURE, "building_patch20", 4)
    with monkeypatch.context() as scoped:
        scoped.setattr(_density_policy, "CANONICAL_ROTATION_TABLE", {})
        atlas = compile_reference_envelopes(snapshot, request).compilation
    _, compilation, spec = _table_spec(compiled)
    donor = next(
        item for item in _lifted(atlas) if item.envelope_spec_id == spec.envelope_spec_id
    )
    assert type(donor.direction_fan_authority) is AdaptiveMinimalRationalFanAuthorityV2
    error = _refused(snapshot, _replace_spec(compilation, spec, donor))
    assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    assert "law applies" in str(error)


def test_the_table_authority_on_an_angle_where_the_law_is_silent_is_refused(
    compiled, monkeypatch
):
    snapshot, compilation, _ = _table_spec(compiled)
    monkeypatch.setattr(_density_policy, "CANONICAL_ROTATION_TABLE", {})
    error = _refused(snapshot, compilation)
    assert error.outcome is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    assert "does not apply" in str(error)
    assert "NO_CANONICAL_ROTATION_TABLE_ENTRY" in str(error)


def test_the_wire_structure_of_the_authority_is_checked_without_geometry(compiled):
    _, _, spec = _table_spec(compiled)
    assert adaptive_density_structure_errors(spec) == ()
    authority = spec.direction_fan_authority
    atlas_law = AdaptiveBoundHiddenSupportDirectionLawV2.ADAPTIVE_MINIMAL_RATIONAL_FAN_V2
    malformed = (
        replace(
            spec,
            hidden_supports=frozenset(
                replace(item, direction_law=atlas_law) for item in spec.hidden_supports
            ),
        ),
        _forge_authority(spec, proven_predicates=frozenset()),
        _forge_authority(spec, ray_rotation_pairs=((12, 5), (1, 1), (5, 13))),
        _forge_authority(spec, hidden_edge_count=2),
        _forge_authority(spec, max_subturn_q=9),
        _forge_authority(
            spec,
            bound_primitive_integer_vectors=((2, 4), *authority.bound_primitive_integer_vectors[1:]),
        ),
        replace(
            spec,
            direction_fan_authority=replace(authority, authority_id="another-authority"),
        ),
    )
    for forged in malformed:
        assert adaptive_density_structure_errors(forged)


def test_the_spec_with_the_table_authority_survives_the_wire_codec(compiled):
    _, _, spec = _table_spec(compiled)
    data = to_canonical_data(spec)
    assert data["direction_fan_authority"]["$type"] == "CanonicalRationalRotationFanAuthorityV1"
    decoded = _decode_as(data, AdaptiveDensityAngularEnvelopeSpecV2)
    assert decoded == spec


# --------------------------------------------------------------------------
# 4. Отказ закона назван: и счётчик, и диагностика, и прежний путь
# --------------------------------------------------------------------------


def _refusals(compilation):
    return [
        item.message.split(":")[0]
        for item in compilation.diagnostics
        if item.outcome is ReferenceOutcome.CANONICAL_FAN_RAYS_LAW_NOT_APPLIED
    ]


def test_a_missing_table_entry_is_named_and_the_old_atlas_decides(monkeypatch):
    snapshot, request = _load(EXACT_FIXTURE, "building_patch20", 4)
    monkeypatch.setattr(_density_policy, "CANONICAL_ROTATION_TABLE", {})
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT
    lifted = _lifted(result.compilation)
    assert lifted
    assert all(
        type(item.direction_fan_authority) is AdaptiveMinimalRationalFanAuthorityV2
        for item in lifted
    )
    assert _refusals(result.compilation) == [
        CanonicalFanRaysRefusalV1.NO_CANONICAL_ROTATION_TABLE_ENTRY.value
    ] * len(lifted)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT"
    assert prepared.counter("CONVEYOR_CANONICAL_FAN_RAYS_LAW_REFUSED") == len(lifted)
    assert prepared.counter("CONVEYOR_CANONICAL_FAN_RAYS_FANS") == 0


def test_a_ray_that_is_irrational_in_the_chart_is_named_and_the_old_atlas_decides(
    monkeypatch,
):
    """Карта с нерациональным корнем из определителя Грама: лучи таблицы иррациональны.

    В полевых слепках определитель Грама всегда полный квадрат, поэтому условие
    имитируется ответом предиката рациональности: закон обязан назвать причину
    и уступить атласу, а не поставить луч без рационального ковектора.
    """

    snapshot, request = _load(EXACT_FIXTURE, "mesh2_patch2", 4)
    monkeypatch.setattr(
        direction_binding, "has_rational_density_support_direction", lambda *_: False
    )
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT
    assert _refusals(result.compilation) == [
        CanonicalFanRaysRefusalV1.CANONICAL_RAYS_IRRATIONAL_IN_CHART.value
    ]


def test_a_fan_that_breaks_the_subturn_guarantee_is_named_and_not_placed(monkeypatch):
    """Ряд с сектором шире `pi/q` — отказ на точной проверке, а не допуск."""

    snapshot, request = _load(EXACT_FIXTURE, "mesh2_patch2", 4)
    wide = {(Fraction(1, 2), 4, 6): ((1, 1), (1, 2), (1, 3))}
    monkeypatch.setattr(_density_policy, "CANONICAL_ROTATION_TABLE", wide)
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT
    assert _refusals(result.compilation) == [
        CanonicalFanRaysRefusalV1.CANONICAL_ROTATION_FAN_VIOLATES_SUBTURN_GUARANTEE.value
    ]
    assert not any(
        type(item.direction_fan_authority) is CanonicalRationalRotationFanAuthorityV1
        for item in _lifted(result.compilation)
    )


def test_the_law_is_silent_on_an_honest_near_right_angle_and_names_nothing():
    """Честный околопрямой угол 90.56 градуса — не канон: закон молчит и не отказывает."""

    snapshot, request = _load(NOISE_FIXTURE, "building004_patch0", 4)
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT
    assert _refusals(result.compilation) == []
    assert not any(
        type(getattr(item, "direction_fan_authority", None))
        is CanonicalRationalRotationFanAuthorityV1
        for item in _specs(result.compilation)
    )


def test_the_decision_is_made_once_per_spec_and_count(compiled):
    snapshot, compilation, spec = _table_spec(compiled)
    context = _context(snapshot, compilation)
    selection = next(iter(compilation.profile_selection_certificates))
    first = canonical_fan_rays_decision(context, spec, selection)
    assert first.authority == spec.direction_fan_authority
    assert canonical_fan_rays_decision(context, spec, selection) is first


# --------------------------------------------------------------------------
# 5. Остальные плотности не тронуты ни байтом
# --------------------------------------------------------------------------


#: Каждый слепок с запросом на плотности ниже d4: что есть в корпусе, то и сверяется.
OTHER_DENSITY_CASES = tuple(
    (folder, name, density)
    for folder, name in CANONICAL_CASES
    for density in (0, 1, 2, 3)
    if (folder / name / f"decal_request_density{density}.json").exists()
)


@pytest.mark.parametrize("folder,name,density", OTHER_DENSITY_CASES)
def test_other_densities_are_byte_identical_with_and_without_the_law(
    monkeypatch, folder, name, density
):
    """Закон работает только на тугом пороге d4: d2 и d3 равны прежним до байта."""

    assert density in (2, 3)
    snapshot, request = _load(folder, name, density)
    with_law = compile_reference_envelopes(snapshot, request)
    with monkeypatch.context() as scoped:
        scoped.setattr(_density_policy, "CANONICAL_ROTATION_TABLE", {})
        without = compile_reference_envelopes(snapshot, request)
    assert with_law.outcome is without.outcome
    assert with_law.compilation == without.compilation
    assert _refusals(with_law.compilation) == []
