"""Аффинный сертификат перетаскивания ширины (`materialize.parametric`): значение и ТОЧНЫЙ наклон из одного прогона.

Что утверждается, и чем оно проверено:

1. ТЕОРЕМА. Для домена без названных причин: значение при `alpha0` плюс наклон на сдвиг РАВНЫ точному прогону на `alpha0 + d` внутри заверенного интервала (при той же структуре) для КАЖДОЙ
   вершины (точка карты и точный подъём) и КАЖДОЙ петли UV `(s, r)` - равенством `SqrtSumV1`, без допуска (`parametric_theorem.theorem_differences`). Полевые домены
   (`building`, `cover.008`, `rounded_wall_noise_top`) судит тот же код в полевом зонде; здесь - фикстуры ядра, в том числе узлы резки (развёртки) и перекладины.
2. НАКЛОН НЕ ПОДГОНЯЕТСЯ. Float-наклоны - округление точных; модель `значение + наклон * сдвиг` против настоящего батча на сдвинутой ширине - округление (1e-9 м и единиц UV на фикстурах).
3. ОТВЕТ НЕ ДВИГАЕТСЯ. Сертификат считается после ответа на своём бюджете: батч, счётчики (все, включая `EXACT_WORK_*`), диагностики и дайджесты те же побитово с сертификатом и без него.
4. НЕАФФИННОЕ НАЗВАНО. Развёртка (смещение по нормали вершины), интервал не заверен, точка покрытия с двумя наклонами, шаблона нет: домен получает сертификат без чисел и с именем `PARAM_*`.
5. ВЫКЛЮЧАТЕЛЬ. `CFTUV_PARAMETRIC=0` - сертификата нет, ответ тот же.
"""

from __future__ import annotations

from fractions import Fraction
from types import SimpleNamespace

import pytest

import developable_factories as df
import materialize_factories as factories
import parametric_theorem as theorem
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.materialize import parametric
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.step import answer_differences, step_domain
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.coverage_template import CutPointV1, FaceTemplateV1, KIND_CUT
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

ROUTE = ("r0a", "r0b")
HOST_LAWS = (NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
CERTIFIED_LAWS = (NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1)

#: Допуск float-кадра против настоящего батча (метры, единицы UV): одно округление точного, позиция хоста и перенос узлов - константы.
FRAME_TOLERANCE = 1e-9


def developable(make, alpha="1"):
    snapshot, request = developable_domain(make(), ROUTE, alpha=alpha)
    prepared, _coverage = prepare_and_cover(snapshot, request)
    return prepared


def field(name):
    snapshot, request = factories.load_fixture(name)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    return prepared


#: `(имя, построитель, законы, alpha0 либо None - ширина запроса, узлы резки ожидаются)`.
CASES = (
    ("fold", lambda: developable(df.fold_strip), CERTIFIED_LAWS, "0.3", True),
    ("slant", lambda: developable(df.slant_fold), CERTIFIED_LAWS, "0.7", True),
    ("quarter", lambda: developable(df.quarter_cylinder), CERTIFIED_LAWS, "0.7", True),
    ("building_002", lambda: field("building_002_full_selection_v1"), HOST_LAWS, None, False),
    ("contact", lambda: field("building_002_point_contact_v1"), HOST_LAWS, None, False),
    ("weighted_normals", lambda: field("building_002_weighted_normals_v1"), HOST_LAWS, None, False),
    ("mesh2_fans", lambda: field("mesh2_patch0_cut_fans_v1"), HOST_LAWS, None, False),
    ("sagging", lambda: field("sagging_wall_convex_partition_v1"), HOST_LAWS, None, False),
)
IDS = [item[0] for item in CASES]


def step(prepared, alpha, laws, *, exact=True, ordered=True):
    return step_domain(
        prepared,
        str(alpha),
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=laws[0],
        decal_topology_law=laws[1],
        digests=False,
        parametric=ordered,
        parametric_exact=exact,
    )


def alpha_of(prepared, given):
    return str(prepared.requested_alpha.value) if given is None else given


def shifts(base, alpha0):
    """Ширины внутри заверенного интервала: в обе стороны на 0.1 %, 1 % и на долю расстояния до края (десятичная запись, как её принимает ядро)."""

    interval = base.result.interval
    found = []
    for share in (Fraction(1, 1000), Fraction(1, 100), Fraction(1, 5)):
        found.append(alpha0 * (1 + share))
        found.append(alpha0 * (1 - share))
    reach_up = None if interval.high is None else (Fraction(interval.high) - alpha0) / 2
    reach_down = (alpha0 - Fraction(interval.low)) / 2
    if reach_up is not None and reach_up > 0:
        found.append(alpha0 + reach_up)
    if reach_down > 0:
        found.append(alpha0 - reach_down)
    texts = []
    for value in found:
        text = theorem.decimal_text(value, 9)
        if Fraction(text) > 0 and Fraction(text) != alpha0 and interval.contains_exact(Fraction(text)):
            texts.append(text)
    return texts


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_value_plus_slope_times_shift_is_the_exact_run(name, build, laws, given, clipped):
    prepared = build()
    text = alpha_of(prepared, given)
    base = step(prepared, text, laws)
    certificate = base.result.parametric
    assert certificate is not None and theorem.usable(certificate), (name, certificate and certificate.markers)
    alpha0 = Fraction(text)
    judged = skipped = 0
    for other_text in shifts(base, alpha0):
        other = step(prepared, other_text, laws)
        outcome, differences = theorem.theorem_differences(base, other, alpha0, Fraction(other_text))
        assert not differences, (name, other_text, differences[:5])
        if outcome == theorem.JUDGED:
            judged += 1
            # Модель `значение + наклон * сдвиг` против настоящего батча: округление.
            errors = theorem.float_errors(base, other, alpha0, Fraction(other_text))
            assert errors is not None and max(errors) <= FRAME_TOLERANCE, (name, other_text, errors)
        else:
            skipped += 1
    assert judged >= 1, f"{name}: no width kept the structure ({skipped} skipped): the theorem was not exercised"
    if clipped:
        assert dict(certificate.counters)["PARAMETRIC_CLIP_NODE_SLOPES"] >= 1, f"{name}: the clip path was not exercised"


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_the_float_slopes_are_the_rounded_exact_ones(name, build, laws, given, clipped):
    prepared = build()
    base = step(prepared, alpha_of(prepared, given), laws)
    certificate = base.result.parametric
    exact = certificate.exact
    assert certificate.vertex_count == len(exact.vertex_keys) and certificate.loop_count == len(exact.loops0)
    assert len(certificate.position_slopes) == 3 * certificate.vertex_count and len(certificate.loop_slopes) == 2 * certificate.loop_count
    for index, slopes in enumerate(exact.position_slopes):
        for axis in range(3):
            assert certificate.position_slopes[index * 3 + axis] == parametric._float(slopes[axis])
    for index, slopes in enumerate(exact.loop_slopes):
        for axis in range(2):
            assert certificate.loop_slopes[index * 2 + axis] == parametric._float(slopes[axis])


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES[:5], ids=IDS[:5])
def test_the_certificate_does_not_move_the_answer(name, build, laws, given, clipped):
    prepared = build()
    text = alpha_of(prepared, given)
    with_certificate = step(prepared, text, laws, exact=False)
    without = step(prepared, text, laws, exact=False, ordered=False)
    assert with_certificate.result.parametric is not None and without.result.parametric is None
    assert answer_differences(with_certificate.result, without.result) == ()
    assert with_certificate.result.counters == without.result.counters  # включая все статьи EXACT_WORK_*
    assert with_certificate.result.interval == without.result.interval and with_certificate.result.structure == without.result.structure


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES[:5], ids=IDS[:5])
def test_the_certificate_spends_no_exact_work_outside_its_own_budget(name, build, laws, given, clipped):
    """Полевые ворота требуют нуля неучтённой работы (`UNBUDGETED_WORK`); сертификат считает на `unlimited_reference_budget`, а ответ и статьи `EXACT_WORK_*` домена те же."""

    from cftuv_envelope.exact_sqrt_sum import UNBUDGETED_WORK, reset_unbudgeted_work

    prepared = build()
    text = alpha_of(prepared, given)
    step(prepared, text, laws, exact=False, ordered=False)  # запись шаблона и сертификата шага
    reset_unbudgeted_work()
    with_certificate = step(prepared, text, laws, exact=True)
    assert UNBUDGETED_WORK.spent == 0, "the certificate leaked exact work past a named budget"
    without = step(prepared, text, laws, exact=False, ordered=False)
    work = [item for item in with_certificate.result.counters if item[0].startswith("EXACT_WORK_")]
    assert work and work == [item for item in without.result.counters if item[0].startswith("EXACT_WORK_")]


def test_a_developable_domain_carries_its_numbers_and_is_named_approximate():
    """PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1: позиции батча развёртки аффинны и отдаются числами, смещение хоста по нормалям - нет, и это названо в `approximate`, а не в `markers`."""

    prepared = developable(df.quarter_cylinder)
    plain = step(prepared, "0.7", CERTIFIED_LAWS, exact=False)
    found = plain.result.parametric
    assert found.markers == () and found.approximate == (parametric.PARAM_DEVELOPABLE_OFFSET,)
    assert found.vertex_count > 0 and found.loop_count > 0 and not found.is_affine
    assert len(found.position_slopes) == 3 * found.vertex_count and len(found.loop_slopes) == 2 * found.loop_count
    # Точный режим несёт те же float-числа плюс точные величины для теоремы.
    judged = step(prepared, "0.7", CERTIFIED_LAWS)
    assert judged.result.parametric.approximate == found.approximate and judged.result.parametric.exact is not None
    assert list(judged.result.parametric.position_slopes) == list(found.position_slopes)


@pytest.mark.parametrize("name,build,alpha", [("quarter", lambda: developable(df.quarter_cylinder), "0.7"), ("fold", lambda: developable(df.fold_strip), "0.3"), ("slant", lambda: developable(df.slant_fold), "0.7")])
def test_the_normal_rate_of_a_developable_domain_is_the_first_order_derivative_of_its_own_offset_normals(name, build, alpha):
    """`approximate_rate` (1/м) - наибольшая `|d n / d alpha|` нормалей смещения движущихся вершин: сверка с конечной разностью НОРМАЛЕЙ САМОГО ЯДРА на двух ширинах одной структуры."""

    import math

    prepared = build()
    delta = Fraction(1, 10_000)
    base = step(prepared, alpha, CERTIFIED_LAWS, exact=False)
    other = step(prepared, theorem.decimal_text(Fraction(alpha) + delta, 6), CERTIFIED_LAWS, exact=False)
    assert base.result.structure.digest == other.result.structure.digest, f"{name}: the structure changed between the two widths"
    first, second = dict(base.result.vertex_normals), dict(other.result.vertex_normals)
    keys = sorted(set(first) & set(second))
    measured = max(math.dist(first[key], second[key]) for key in keys) / float(delta)
    rate = base.result.parametric.approximate_rate
    assert rate > 0.0, f"{name}: the normals of the domain do not move, nothing to compare"
    assert abs(measured - rate) <= 0.02 * rate + 1e-6, (name, measured, rate)
    # Аффинный домен нормалей смещения по вершинам не несёт.
    assert step(field("building_002_point_contact_v1"), "0.25", HOST_LAWS, exact=False).result.parametric.approximate_rate == 0.0


def test_an_affine_domain_names_nothing_approximate():
    prepared = field("building_002_point_contact_v1")
    found = step(prepared, str(prepared.requested_alpha.value), HOST_LAWS, exact=False).result.parametric
    assert found.markers == () and found.approximate == () and found.is_affine


def test_an_uncertified_window_is_named():
    prepared = field("building_002_point_contact_v1")
    base = step(prepared, str(prepared.requested_alpha.value), HOST_LAWS)
    moved = SimpleNamespace(status="AT_EVENT", reason="")
    # `certificate_of` не читает ничего, кроме `alpha`/`scale` входов, пока окна нет: отказ называется до работы.
    inputs = SimpleNamespace(alpha=Fraction(1, 4), scale=1)
    request = parametric.ParametricRequestV1({}, False)
    found = parametric.certificate_of(inputs, base.result.batch, moved, request, False, prepared, None)
    assert found.markers == (parametric.PARAM_WINDOW_NOT_CERTIFIED,) and not found.is_affine


def test_a_missing_template_or_contour_is_named():
    prepared = field("building_002_point_contact_v1")
    region = SimpleNamespace(region_id=prepared.regions[0].region_id, contours=None)
    with pytest.raises(parametric.ParametricRefusal) as raised:
        parametric.point_slopes_of(prepared, SimpleNamespace(regions=(region,)), {})
    assert raised.value.name == parametric.PARAM_CONTOURS_UNAVAILABLE
    region = SimpleNamespace(region_id=prepared.regions[0].region_id, contours=())
    with pytest.raises(parametric.ParametricRefusal) as raised:
        parametric.point_slopes_of(prepared, SimpleNamespace(regions=(region,)), {})
    assert raised.value.name == parametric.PARAM_TEMPLATE_UNAVAILABLE


def test_one_point_with_two_slopes_is_named():
    prepared = field("building_002_point_contact_v1")
    one = SqrtSumV1.rational(1)
    other = SqrtSumV1.rational(2)
    point = (SqrtSumV1.rational(3), SqrtSumV1.rational(4))
    partition = prepared.regions[0].partition
    template = SimpleNamespace(
        faces=(
            FaceTemplateV1("a", KIND_CUT, (CutPointV1(point[0], point[1], one, one),), None, 1),
            FaceTemplateV1("b", KIND_CUT, (CutPointV1(point[0], point[1], other, other),), None, 1),
        )
    )
    contours = (SimpleNamespace(points=(point,)), SimpleNamespace(points=(point,)))
    region = SimpleNamespace(region_id=prepared.regions[0].region_id, contours=contours)
    with pytest.raises(parametric.ParametricRefusal) as raised:
        parametric.point_slopes_of(prepared, SimpleNamespace(regions=(region,)), {id(partition): template})
    assert raised.value.name == parametric.PARAM_POINT_SLOPE_AMBIGUOUS


def test_the_switch_turns_the_certificate_off(monkeypatch):
    prepared = field("building_002_point_contact_v1")
    monkeypatch.setenv(parametric.ENVIRONMENT_SWITCH, "0")
    off = step(prepared, str(prepared.requested_alpha.value), HOST_LAWS, exact=False)
    assert off.result.parametric is None


def test_every_marker_of_the_kernel_is_named_once_and_listed():
    """A refusal has a name: every `PARAM_*` constant of the module is listed in `MARKERS` exactly once and its value is a `PARAM_*` name (a marker nobody lists is a silent refusal)."""
    constants = [value for name, value in vars(parametric).items() if name.startswith("PARAM_") and isinstance(value, str)]
    assert constants, "the module names no marker"
    assert all(value.startswith("PARAM_") for value in constants)
    assert sorted(parametric.MARKERS) == sorted(constants)
    assert len(set(parametric.MARKERS)) == len(parametric.MARKERS)
