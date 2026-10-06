"""Пропуск точной проверки аффинности UV у простых граней (`assemble.station_values` -> `plain`, `_plane_ring` -> `affine_known`).

`(s, r)` вершины грани - `station_of` и `transverse_of` её пробега и прямой: линейные по точке формулы, поэтому UV такой грани ТОЧНО
аффинна по построению, а `uv_is_affine_in_chart` (произведения иррациональных значений на каждую вершину) возвращает то же `True`. Грань
простая, только когда факты ВСЕХ вершин её контура в итоге равны её собственным значениям: конфликт, перекладина JOIN, станция по хорде
и разомкнутое кольцо выводят её из множества, и проверка идёт.

Что проверено: (1) ответ с пропуском равен ответу с принудительной проверкой (`VERIFY_PLAIN_AFFINE`) - батч, нормали, диагностики и все
счётчики ответа (статьи бюджета - цена, и пропуск её уменьшает); принудительная проверка у простой грани обязана сойтись, иначе
`AssertionError`; (2) пропуск пропускает: проверок меньше; (3) пропуск не безусловный: у граней с перекладинами проверка остаётся.
"""

from __future__ import annotations

import pytest

import developable_factories as df
import materialize_factories as factories
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.materialize import assemble as assemble_module
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.step import answer_differences, _full
from cftuv_envelope.wavefront import prepare_conveyor

HOST_LAWS = (NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)


def field(name):
    snapshot, request = factories.load_fixture(name)
    return prepare_conveyor(snapshot, request)


def developable(make, alpha="1"):
    snapshot, request = developable_domain(make(), ("r0a", "r0b"), alpha=alpha)
    prepared, _ = prepare_and_cover(snapshot, request)
    return prepared


CASES = (
    ("building_002", lambda: field("building_002_full_selection_v1"), None),
    ("mesh2_fans", lambda: field("mesh2_patch0_cut_fans_v1"), None),
    ("noise_top_rungs", lambda: field("wall_noise_top_rung_clip_v1"), None),
    ("sagging", lambda: field("sagging_wall_convex_partition_v1"), None),
    ("quarter", lambda: developable(df.quarter_cylinder), "0.7"),
)


def run(prepared, alpha, verify, monkeypatch):
    calls = []
    original = assemble_module.uv_is_affine_in_chart

    def counting(points, values, budget):
        calls.append(1)
        return original(points, values, budget)

    monkeypatch.setattr(assemble_module, "uv_is_affine_in_chart", counting)
    monkeypatch.setattr(assemble_module, "VERIFY_PLAIN_AFFINE", verify)
    laws = HOST_LAWS
    request = materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")
    result = _full(prepared, str(alpha), (request, laws[0], laws[1], False))
    return result, len(calls)


def answer_counters(result):
    return tuple(item for item in result.counters if not item[0].startswith(("EXACT_WORK", "MATERIALIZE_EXACT")))


@pytest.mark.parametrize("name,build,given", CASES, ids=[item[0] for item in CASES])
def test_the_skip_gives_the_same_answer_as_the_forced_check(name, build, given, monkeypatch):
    prepared = build()
    alpha = prepared.requested_alpha.value if given is None else given
    skipped, skipped_checks = run(prepared, alpha, False, monkeypatch)
    forced, forced_checks = run(prepared, alpha, True, monkeypatch)  # raises if a plain face is not affine
    assert skipped.is_materialized and forced.is_materialized
    assert tuple(item for item in answer_differences(skipped, forced) if item != "counters") == ()
    assert answer_counters(skipped) == answer_counters(forced)
    assert skipped_checks <= forced_checks
    if name in ("building_002", "mesh2_fans", "quarter"):
        assert skipped_checks < forced_checks, "plain faces exist here and must skip the check"


def test_the_check_stays_where_a_rung_changed_the_facts(monkeypatch):
    prepared = field("wall_noise_top_rung_clip_v1")
    skipped, skipped_checks = run(prepared, prepared.requested_alpha.value, False, monkeypatch)
    assert skipped.is_materialized
    rungs = dict(skipped.counters).get("MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX", 0) + dict(skipped.counters).get(
        "MATERIALIZE_RUNG_CHORD_STATIONS", 0
    )
    assert rungs > 0 and skipped_checks > 0, "a face with a rung station is not plain: its affinity is checked exactly"


def test_a_plain_face_with_a_doctored_fact_is_caught_by_the_forced_check(monkeypatch):
    prepared = field("building_002_full_selection_v1")
    original = assemble_module.station_values

    def doctored(*arguments, **keywords):
        facts = original(*arguments, **keywords)
        slot = next(iter(facts))
        s, r = facts[slot]
        facts[slot] = (s + s.__class__.rational(1), r)  # `plain` was computed on the honest values
        return facts

    monkeypatch.setattr(assemble_module, "station_values", doctored)
    # `domain.py` imports the name: patch the importer too.
    from cftuv_envelope.materialize import domain as domain_module

    monkeypatch.setattr(domain_module, "station_values", doctored)
    with pytest.raises(AssertionError):
        run(prepared, prepared.requested_alpha.value, True, monkeypatch)
