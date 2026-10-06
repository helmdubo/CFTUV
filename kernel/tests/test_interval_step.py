"""Шаг ширины домена (`materialize.step`): покрытие из шаблона внутри заверенного интервала равно полному счёту ТОЧНО.

Что утверждается, и чем оно проверено:

1. ПОПАДАНИЕ = ПОЛНЫЙ ПУТЬ. Для полевых доменов (снапшоты полевых выгрузок) и развёрток шаг внутри интервала покрытия (у обоих краёв,
   в серединах половин, шаги 0.1 % и 1 % в обе стороны) отвечает ровно как `conveyor_coverage` + `materialize_domain`: батч (канонические
   байты), счётчики (все, включая статьи бюджета), диагностики, нормали вершин и их дайджесты. Покрытие, воспроизведённое из шаблона, равно
   `coverage_at` грань за гранью и точка за точкой (`SqrtSumV1` каноничны, поэтому равенство значений - равенство побитово).
2. ПРОМАХ НАЗВАН. Ширина за границей интервала, первый счёт, выключатель, отказавший шаблон - полный путь под именем причины, а счётчик
   (`STEP_COUNTERS`) растёт ровно на единицу; ответ и тут равен полному.
3. СВЕРКА ЛОВИТ РАСХОЖДЕНИЕ. Подмена шаблона (`CFTUV_INTERVAL_VERIFY=1`) возвращает ПОЛНЫЙ ответ под именем `VERIFY_MISMATCH:<части>`.
4. ИМЕНА ЭКЗЕМПЛЯРОВ. Окно имён (знаки контактов резолвера границы против запрошенной alpha) отдельно от окна покрытия: за окном имён имена
   считает сам резолвер, ответ тот же; окно считается по оболочкам контактов и схлопывается, когда оболочка накрыла alpha.
5. СЕРТИФИКАТ НЕ ЕДЕТ ПО ТРУБЕ. Пикл подготовки несёт память пустой: сертификат живёт в воркере.
"""

from __future__ import annotations

import dataclasses
import pickle
from fractions import Fraction

import pytest

import developable_factories as df
import materialize_factories as factories
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize import step as step_module
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.memo import memo_of
from cftuv_envelope.materialize.step import (
    ENVIRONMENT_SWITCH,
    ENVIRONMENT_VERIFY,
    PATH_FAST,
    STEP_COUNTERS,
    answer_differences,
    step_domain,
)
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.conveyor import requested_alpha_fraction
from cftuv_envelope.wavefront.coverage import _coverage_at, clear_recent_coverage, current_coverage_source
from cftuv_envelope.wavefront.coverage_template import KIND_CUT

ROUTE = ("r0a", "r0b")
HOST_LAWS = (NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
CERTIFIED_LAWS = (NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1)


def developable(make, alpha="1"):
    snapshot, request = developable_domain(make(), ROUTE, alpha=alpha)
    prepared, _coverage = prepare_and_cover(snapshot, request)
    return prepared


def field(name):
    snapshot, request = factories.load_fixture(name)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    return prepared


#: `(имя, построитель домена, законы, ширина alpha0 либо None - ширина запроса)`.
CASES = (
    ("fold", lambda: developable(df.fold_strip), CERTIFIED_LAWS, "0.3"),
    ("slant-host", lambda: developable(df.slant_fold), HOST_LAWS, "0.7"),
    ("quarter", lambda: developable(df.quarter_cylinder), CERTIFIED_LAWS, "0.7"),
    ("noise_top", lambda: field("wall_noise_top_rung_clip_v1"), HOST_LAWS, None),
    ("building_002", lambda: field("building_002_full_selection_v1"), HOST_LAWS, None),
    ("contact", lambda: field("building_002_point_contact_v1"), HOST_LAWS, None),
    ("mesh2_fans", lambda: field("mesh2_patch0_cut_fans_v1"), HOST_LAWS, None),
    ("sagging", lambda: field("sagging_wall_convex_partition_v1"), HOST_LAWS, None),
)


def request_of(prepared):
    return materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")


def step(prepared, alpha, laws):
    clear_recent_coverage()
    return step_domain(
        prepared,
        str(alpha),
        request=request_of(prepared),
        near_planar_lift_law=laws[0],
        decal_topology_law=laws[1],
        digests=False,
    )


def reference(prepared, alpha, laws):
    """Полный путь без шаблонов: память недавних покрытий сброшена, источника покрытия нет."""

    clear_recent_coverage()
    answer = step_module._full(prepared, str(alpha), (request_of(prepared), laws[0], laws[1], False))
    clear_recent_coverage()
    return answer


def certificate_of(prepared, laws):
    found = memo_of(prepared).fetch(("step", laws[0].value, laws[1].value))
    return found[0] if found else None


def alpha_text(prepared, given):
    return str(prepared.requested_alpha.value) if given is None else given


def inside_widths(certificate):
    """Ширины строго внутри интервала покрытия: у обоих краёв (0.1 % и 2 % до границы), в середине каждой половины, шаги 0.1 % и 1 %."""

    alpha = float(certificate.alpha)
    high = certificate.high if certificate.high is not None else alpha * 3
    found = []
    for edge in (certificate.low, high):
        for share in (0.002, 0.5, 0.98):
            found.append(alpha + (edge - alpha) * share)
    for factor in (0.999, 1.001, 0.99, 1.01):
        found.append(alpha * factor)
    return sorted({round(value, 12) for value in found if certificate.low < value < high and value > 0 and Fraction(value) != certificate.alpha})


def beyond_widths(certificate):
    found = []
    if certificate.low > 0:
        found.append(certificate.low - (certificate.low * 0.002 + 1e-7))
    if certificate.high is not None:
        found.append(certificate.high + (certificate.high * 0.002 + 1e-7))
    return [round(value, 12) for value in found if value > 0]


@pytest.fixture(autouse=True)
def _clean_environment(monkeypatch):
    monkeypatch.delenv(ENVIRONMENT_SWITCH, raising=False)
    monkeypatch.delenv(ENVIRONMENT_VERIFY, raising=False)


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_a_hit_inside_the_interval_answers_exactly_like_the_full_path(name, build, laws, given):
    prepared = build()
    base = alpha_text(prepared, given)
    first = step(prepared, base, laws)
    assert first.path == "FALLBACK:NO_CERTIFICATE", first.path
    assert first.result.is_materialized, first.result.detail
    certificate = certificate_of(prepared, laws)
    assert certificate is not None and certificate.low < float(base) and (certificate.high is None or float(base) < certificate.high)
    assert not answer_differences(first.result, reference(prepared, base, laws))
    widths = inside_widths(certificate)
    assert widths, "the interval must have room for a step"
    hits = 0
    for width in widths:
        hit = step(prepared, width, laws)
        assert hit.is_hit, (name, width, hit.path, certificate.low, certificate.high)
        hits += 1
        assert hit.result.is_materialized
        assert answer_differences(hit.result, reference(prepared, width, laws)) == (), (name, width)
        # Ответ равен полному целиком, включая счётчики (статьи бюджета тоже): остальные стадии идут своим кодом на тех же значениях.
        assert hit.result.counters == reference(prepared, width, laws).counters
    assert hits == len(widths)


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_the_template_reproduces_the_coverage_point_for_point(name, build, laws, given):
    prepared = build()
    base = alpha_text(prepared, given)
    step(prepared, base, laws)
    certificate = certificate_of(prepared, laws)
    scale = 1 if prepared.lattice is None else prepared.lattice.scale
    checked = 0
    for width in inside_widths(certificate)[:5]:
        lattice_alpha = Fraction(str(width)) * scale
        for region in prepared.regions:
            partition = region.partition
            template = certificate.templates[id(partition)]
            built = step_module.instantiate(template, partition, lattice_alpha, None)
            truth = _coverage_at(partition, lattice_alpha, None, None)
            assert built is not None and built.outcome == truth.outcome
            assert built.faces == truth.faces, (name, width)
            assert built.doubled_area == truth.doubled_area
            checked += 1
    assert checked >= 1
    cut_faces = sum(1 for template in certificate.templates.values() for face in template.faces if face.kind == KIND_CUT)
    # `sagging` в этой ширине покрыт целиком (фронт нигде не режет грань): единственный случай без точек отсечения.
    assert (cut_faces >= 1 and certificate.cuts >= 1) or name == "sagging", "a template with cut points at least"


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_outside_the_interval_the_full_path_runs_under_a_named_reason(name, build, laws, given):
    prepared = build()
    base = alpha_text(prepared, given)
    step(prepared, base, laws)
    certificate = certificate_of(prepared, laws)
    seen = set()
    for width in beyond_widths(certificate):
        before = dict(STEP_COUNTERS)
        outside = step(prepared, width, laws)
        reason = outside.path
        assert reason in ("FALLBACK:OUTSIDE_BELOW", "FALLBACK:OUTSIDE_ABOVE", "FALLBACK:OUTSIDE_BETWEEN"), reason
        seen.add(reason)
        assert STEP_COUNTERS["INTERVAL_FALLBACK_" + reason.split(":")[1]] == before.get("INTERVAL_FALLBACK_" + reason.split(":")[1], 0) + 1
        assert answer_differences(outside.result, reference(prepared, width, laws)) == ()
    assert seen, "a domain with no bound at all would be certified for every width"


def test_the_switch_turns_the_fast_path_off_and_names_it(monkeypatch):
    prepared = developable(df.quarter_cylinder)
    monkeypatch.setenv(ENVIRONMENT_SWITCH, "0")
    before = STEP_COUNTERS["INTERVAL_FALLBACK_DISABLED"]
    for width in ("0.7", "0.71"):
        stepped = step(prepared, width, CERTIFIED_LAWS)
        assert stepped.path == "FALLBACK:DISABLED"
        assert answer_differences(stepped.result, reference(prepared, width, CERTIFIED_LAWS)) == ()
    assert STEP_COUNTERS["INTERVAL_FALLBACK_DISABLED"] == before + 2
    assert certificate_of(prepared, CERTIFIED_LAWS) is None, "nothing is recorded while the switch is off"


def test_a_refused_template_is_named_and_stops_costing_after_the_attempt_limit(monkeypatch):
    prepared = developable(df.quarter_cylinder)
    calls = []

    def refuse(*arguments):
        calls.append(1)
        return None

    monkeypatch.setattr(step_module, "build_template", refuse)
    paths = [step(prepared, f"0.{70 + number}", CERTIFIED_LAWS).path for number in range(step_module.TEMPLATE_ATTEMPT_LIMIT + 2)]
    assert paths[: step_module.TEMPLATE_ATTEMPT_LIMIT] == ["FALLBACK:NO_CERTIFICATE"] * step_module.TEMPLATE_ATTEMPT_LIMIT
    assert paths[step_module.TEMPLATE_ATTEMPT_LIMIT :] == ["FALLBACK:TEMPLATE_UNAVAILABLE"] * 2
    assert len(calls) == len(prepared.regions) * step_module.TEMPLATE_ATTEMPT_LIMIT, "no template is built after the limit"
    assert certificate_of(prepared, CERTIFIED_LAWS) is None


def test_a_recording_bug_cannot_break_the_full_path(monkeypatch):
    prepared = developable(df.quarter_cylinder)

    def broken(*_arguments):
        raise RuntimeError("template bug")

    monkeypatch.setattr(step_module, "build_template", broken)
    before = STEP_COUNTERS["INTERVAL_" + step_module.TEMPLATE_ERROR]
    stepped = step(prepared, "0.7", CERTIFIED_LAWS)
    assert stepped.path == "FALLBACK:NO_CERTIFICATE" and stepped.result.is_materialized
    assert STEP_COUNTERS["INTERVAL_" + step_module.TEMPLATE_ERROR] == before + len(prepared.regions)
    assert answer_differences(stepped.result, reference(prepared, "0.7", CERTIFIED_LAWS)) == ()
    assert certificate_of(prepared, CERTIFIED_LAWS) is None


def test_the_verification_returns_the_full_answer_and_names_the_mismatch(monkeypatch):
    prepared = developable(df.quarter_cylinder)
    step(prepared, "0.7", CERTIFIED_LAWS)
    honest = step_module.instantiate

    def corrupted(template, partition, alpha, work_budget):
        found = honest(template, partition, alpha, work_budget)
        if found is None or not found.faces or not found.faces[0].points:
            return found
        # Первая точка первой грани сдвинута на тысячную решётки: покрытие правдоподобно, но не то.
        faces = list(found.faces)
        (x, y), rest = faces[0].points[0], faces[0].points[1:]
        faces[0] = dataclasses.replace(faces[0], points=((x + SqrtSumV1.rational(Fraction(1, 1000)), y), *rest))
        return dataclasses.replace(found, faces=tuple(faces))

    monkeypatch.setattr(step_module, "instantiate", corrupted)
    monkeypatch.setenv(ENVIRONMENT_VERIFY, "1")
    before = STEP_COUNTERS[step_module.VERIFY_MISMATCH]
    stepped = step(prepared, "0.701", CERTIFIED_LAWS)
    assert stepped.path.startswith(step_module.VERIFY_MISMATCH + ":"), stepped.path
    assert STEP_COUNTERS[step_module.VERIFY_MISMATCH] == before + 1
    assert answer_differences(stepped.result, reference(prepared, "0.701", CERTIFIED_LAWS)) == (), "the answer on a mismatch is the full one"
    monkeypatch.setattr(step_module, "instantiate", honest)
    clean = step(prepared, "0.702", CERTIFIED_LAWS)
    assert clean.is_hit, clean.path


def test_the_verification_passes_on_an_honest_hit(monkeypatch):
    prepared = developable(df.fold_strip)
    step(prepared, "0.3", CERTIFIED_LAWS)
    monkeypatch.setenv(ENVIRONMENT_VERIFY, "1")
    before = STEP_COUNTERS[step_module.VERIFY_MISMATCH]
    stepped = step(prepared, "0.301", CERTIFIED_LAWS)
    assert stepped.path == PATH_FAST and STEP_COUNTERS[step_module.VERIFY_MISMATCH] == before


def test_the_names_are_taken_from_the_certificate_inside_their_window_and_computed_outside_it():
    prepared = field("mesh2_patch0_cut_fans_v1")
    base = alpha_text(prepared, None)
    step(prepared, base, HOST_LAWS)
    certificate = certificate_of(prepared, HOST_LAWS)
    assert certificate.names is not None and any(kind is not None for kind in certificate.names.values()), "a shortened strip is in this field"
    taken = []
    original = step_module._Instantiator.instance_ids

    def watch(self, prepared_, alpha_value):
        found = original(self, prepared_, alpha_value)
        taken.append(found is not None)
        return found

    step_module._Instantiator.instance_ids = watch
    try:
        inside = float(certificate.alpha) * 1.002
        assert certificate.names_hold(Fraction(str(inside)))
        hit = step(prepared, inside, HOST_LAWS)
        assert hit.is_hit and taken[-1] is True
        assert answer_differences(hit.result, reference(prepared, inside, HOST_LAWS)) == ()
        # Ширина внутри окна покрытия, но за окном имён: резолвер считает сам, ответ тот же.
        outside = round(float(certificate.names_low) * 0.5 + certificate.low * 0.5, 12)
        assert certificate.low < outside < float(certificate.names_low), "the fixture has a gap between the coverage window and the names window"
        hit = step(prepared, outside, HOST_LAWS)
        assert hit.is_hit and taken[-1] is False
        assert answer_differences(hit.result, reference(prepared, outside, HOST_LAWS)) == ()
    finally:
        step_module._Instantiator.instance_ids = original


def test_the_contact_window_is_the_gap_around_the_width_and_collapses_on_a_covered_width():
    window = step_module._contact_window
    alpha = Fraction(1, 2)
    assert window([], alpha) == (0, None)
    below, above = (Fraction(1, 10), Fraction(1, 10)), (Fraction(9, 10), Fraction(9, 10))
    assert window([below, above, (Fraction(3, 10), Fraction(3, 10))], alpha) == (Fraction(3, 10), Fraction(9, 10))
    assert window([(Fraction(2, 5), Fraction(3, 5))], alpha) is None, "the envelope covers the width: no window"
    assert window([None], alpha) is None, "a contact with no envelope is not certified"
    assert window([(alpha, alpha)], alpha) is None, "a contact exactly at the width"


def test_the_certificate_never_travels_with_the_preparation():
    prepared = developable(df.fold_strip)
    step(prepared, "0.3", CERTIFIED_LAWS)
    assert certificate_of(prepared, CERTIFIED_LAWS) is not None
    clone = pickle.loads(pickle.dumps(prepared))
    assert certificate_of(clone, CERTIFIED_LAWS) is None
    assert step(clone, "0.31", CERTIFIED_LAWS).path == "FALLBACK:NO_CERTIFICATE"


def test_without_a_source_the_coverage_is_the_old_one_and_the_hook_is_inert():
    assert current_coverage_source() is None
    prepared = developable(df.fold_strip)
    plain = reference(prepared, "0.3", CERTIFIED_LAWS)
    again = reference(prepared, "0.3", CERTIFIED_LAWS)
    assert answer_differences(plain, again) == ()


def test_the_requested_alpha_is_read_exactly_as_the_coverage_reads_it():
    assert requested_alpha_fraction("0.2239") == Fraction(2239, 10000)
    assert requested_alpha_fraction(1) == Fraction(1)
    with pytest.raises(TypeError):
        requested_alpha_fraction(0.25)
