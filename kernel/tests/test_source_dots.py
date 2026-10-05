"""Закон `SILHOUETTE_SOURCE_DOTS_V1` (срез S4): точки на прямых цепях источника и стены, общее решение по всем доменам прогона.

Что здесь доказано и чем.

* СИНТЕТИКА (полоса вдоль цепи источника, батчи собраны руками и проходят `validate_geometry_batch` и `audit_batch`): точка на
  прямой цепи растворяется в одном домене и в двух доменах сразу (общая цепь: одни и те же `location:src:`), цепи итога — те же,
  что дал бы `paths_of` на рёбрах без точки; излом цепи, вершина с прикреплённым ребром, сдвиг UV сверх допуска, хорда сверх
  бюджета, сгиб между гранями соседей оставляют место ВО ВСЕХ доменах и каждая названа счётчиком, в том числе `KEPT_OTHER_DOMAIN`
  у домена, где точка прошла бы сама.
* ПРОВЕРКА (`verify_source_dots`) ловит испорченный итог: красные контроли — растворённый угол цепи, место, растворённое лишь в
  одном домене, сдвиг и хорда сверх записанных максимумов, устаревший дайджест.
* ПОЛНОТА: порядок доменов на ответ не влияет; прогон без точек не меняет ни батч, ни число.
"""

from __future__ import annotations

import dataclasses
import math
from fractions import Fraction
from pathlib import Path

import pytest
from source_dots_factories import NORMAL, XS, strip as _strip

from cftuv_envelope.materialize import source_dots
from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.source_dots import SourceDotInputV1, reconcile_source_dots, verify_source_dots
from cftuv_envelope.validation import validate_geometry_batch

SLIDE = Fraction(1, 256)


def _input(key, batch):
    return SourceDotInputV1(key, batch, NORMAL, ())


def _sound(batch):
    assert not validate_geometry_batch(batch)
    assert not audit_batch(batch, NORMAL).problems()
    return batch


def _keys(batch, kind="SOURCE"):
    return [[key.value for key in chain.ordered_vert_keys] for chain in batch.boundary_chains if chain.semantic_boundary_id.value.split(":")[1] == kind]



def test_the_synthetic_strips_are_sound_batches():
    for side in (1, -1):
        batch = _sound(_strip("a", XS, side=side))
        assert _keys(batch) == [["src:s0", "src:s1", "src:s2", "src:s3", "src:s4"]] or _keys(batch) == [["src:s4", "src:s3", "src:s2", "src:s1", "src:s0"]]


def test_the_dots_of_one_domain_on_a_straight_chain_are_dissolved_and_the_result_is_a_sound_batch():
    batch = _sound(_strip("a", XS))

    result = reconcile_source_dots([_input(0, batch)], SLIDE)

    (domain,) = result.domains
    assert result.changed and not result.problems
    assert domain.removed == {"src:s1", "src:s2", "src:s3"}
    assert dict(domain.counters)[source_dots.DISSOLVED] == 3
    assert _keys(domain.batch) == [["src:s0", "src:s4"]]
    assert not validate_geometry_batch(domain.batch)
    assert dict(domain.overrides)["MATERIALIZE_VERTICES"] == len(batch.vertices) - 3
    assert len(domain.batch.faces[0].ordered_vert_keys) == len(batch.faces[0].ordered_vert_keys) - 3
    assert not verify_source_dots([_input(0, batch)], result, SLIDE)


def test_a_shared_chain_is_dissolved_in_both_domains_at_once_and_the_anchors_agree():
    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1))

    result = reconcile_source_dots([_input(0, one), _input(1, other)], SLIDE)

    assert result.changed and not result.problems
    assert [domain.removed for domain in result.domains] == [{"src:s1", "src:s2", "src:s3"}] * 2
    assert dict(result.domains[0].counters)[source_dots.SHARED_DISSOLVED] == 3
    first, second = (_keys(domain.batch)[0] for domain in result.domains)
    assert sorted(first) == sorted(second) == ["src:s0", "src:s4"]
    assert not verify_source_dots([_input(0, one), _input(1, other)], result, SLIDE)


def test_a_corner_of_the_chain_is_not_a_dot_and_is_never_dissolved_or_counted():
    batch = _sound(_strip("a", XS, bent=(2,)))

    result = reconcile_source_dots([_input(0, batch)], SLIDE)

    assert result.domains[0].removed == set() or "src:s2" not in result.domains[0].removed
    assert "src:s2" in _keys(result.domains[0].batch)[0]


def _run(*batches):
    inputs = [_input(number, batch) for number, batch in enumerate(batches)]
    return inputs, reconcile_source_dots(inputs, SLIDE)


def _counted(domain, name):
    return dict(domain.counters).get(name, 0)


def test_a_vertex_with_an_attached_edge_in_one_domain_keeps_the_place_in_both_and_every_point_is_named():
    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1, rung=2))

    inputs, result = _run(one, other)

    assert result.changed and not result.problems
    assert [domain.removed for domain in result.domains] == [{"src:s1", "src:s3"}] * 2
    assert _counted(result.domains[0], source_dots.KEPT_OTHER_DOMAIN) == 1  # у `a` точка прошла бы сама
    assert _counted(result.domains[1], source_dots.KEPT_ATTACHED) == 1
    assert "src:s2" in _keys(result.domains[0].batch)[0]
    assert not verify_source_dots(inputs, result, SLIDE)


def test_a_place_whose_neighbours_along_the_chain_differ_between_the_domains_is_kept_by_name():
    """У одного домена между `src:s1` и `src:s2` стоит своя вершина `node:`: соседи `s1` и `s2` у доменов разные, и решить место одинаково нельзя."""

    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1, local={1: "node:between"}))

    inputs, result = _run(one, other)

    assert {"src:s1", "src:s2"}.isdisjoint(set().union(*(domain.removed for domain in result.domains)))
    assert _counted(result.domains[0], source_dots.KEPT_NEIGHBOURS_DIFFER) == 2
    assert result.domains[0].removed == {"src:s3"} == result.domains[1].removed  # s3 и в обоих доменах соседи те же
    assert not verify_source_dots(inputs, result, SLIDE)


def test_a_vertex_on_an_interface_chain_is_not_a_dot_of_the_boundary_and_stays_by_name():
    from cftuv_envelope.canonical import geometry_batch_semantic_digest
    from cftuv_envelope.contracts.geometry_batch import GeometryInterfaceChainV1
    from cftuv_envelope.ids import SemanticDigestValue, SemanticInterfaceId, VertexKey

    batch = _strip("a", XS)
    seam = GeometryInterfaceChainV1(SemanticInterfaceId("interface:0:1:0"), (VertexKey("src:s2"), VertexKey("node:a1")))
    batch = dataclasses.replace(batch, interface_chains=frozenset({seam}))
    batch = dataclasses.replace(batch, semantic_digest=SemanticDigestValue(geometry_batch_semantic_digest(batch).sha256_hex))

    inputs, result = _run(_sound(batch))

    assert "src:s2" not in result.domains[0].removed and {"src:s1", "src:s3"} <= result.domains[0].removed
    assert _counted(result.domains[0], source_dots.KEPT_ATTACHED) == 1


def test_a_foreign_vertex_or_edge_of_the_face_within_the_chord_depth_of_the_new_edge_keeps_the_dot_by_name():
    """Полоса шириной 3 мм: фронт грани лежит в глубине хорды от выпрямленного ребра, и слияние ребра с ним не было бы простым."""

    inputs, result = _run(_sound(_strip("a", XS, height=0.003)))

    assert not result.changed and _counted(result.domains[0], source_dots.KEPT_NOT_SIMPLE) == 3


def test_a_place_that_is_the_end_of_the_chain_in_the_neighbour_domain_is_kept_by_name_and_its_other_dots_go():
    """Соседний домен покрывает цепь лишь до `src:s2`: там это конец цепи (не точка), решение места — «оставить»; `s1` общий и растворяется, `s3` свой."""

    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS[:3], side=-1))

    inputs, result = _run(one, other)

    assert result.domains[0].removed == {"src:s1", "src:s3"} and result.domains[1].removed == {"src:s1"}
    assert _counted(result.domains[0], source_dots.KEPT_OTHER_DOMAIN) == 1
    assert not verify_source_dots(inputs, result, SLIDE)


def test_a_place_with_a_twin_copy_in_the_domain_is_never_dissolved():
    """Двойник-копия места (разрез кольца): у двух вершин одна ссылка, и растворить одну значило бы разорвать место."""

    batch = _sound(_strip("a", XS, local={1: "node:twin"}, refs={"node:twin": "location:src:s2"}))

    inputs, result = _run(batch)

    assert "src:s2" not in result.domains[0].removed and "node:twin" not in result.domains[0].removed
    assert _counted(result.domains[0], source_dots.KEPT_ATTACHED) >= 1
    assert not verify_source_dots(inputs, result, SLIDE)


def test_the_dots_of_a_closed_chain_are_dissolved_including_the_one_where_the_closed_path_starts_and_ends():
    from source_dots_factories import square

    batch = _sound(square("q", per_side=3))
    (closed,) = _keys(batch)
    assert closed[0] == closed[-1] == "src:a01"  # путь замкнутой цепи начинается и кончается в точке

    inputs, result = _run(batch)

    (domain,) = result.domains
    assert domain.removed == {key for key in (key.value for key in batch.faces[0].ordered_vert_keys) if key.startswith("src:a")}
    (chain,) = _keys(domain.batch)
    assert chain[0] == chain[-1] and sorted(set(chain)) == ["src:z0", "src:z1", "src:z2", "src:z3"]
    assert len(domain.batch.faces[0].ordered_vert_keys) == 4 and not validate_geometry_batch(domain.batch)
    assert not verify_source_dots(inputs, result, SLIDE)


def test_a_uv_slide_beyond_the_request_keeps_the_place_in_both_domains_by_name():
    one, other = _sound(_strip("a", XS, skew={2: 0.1})), _sound(_strip("b", XS, side=-1))

    inputs, result = _run(one, other)

    assert not result.changed
    assert _counted(result.domains[0], source_dots.KEPT_UV) == 3
    assert _counted(result.domains[1], source_dots.KEPT_OTHER_DOMAIN) == 3
    assert result.domains[0].batch is one and result.domains[1].batch is other


def test_a_looser_request_dissolves_what_a_strict_one_keeps():
    one = _sound(_strip("a", XS, skew={2: 0.003}))
    inputs = [_input(0, one)]

    strict = reconcile_source_dots(inputs, Fraction(1, 1000))
    loose = reconcile_source_dots(inputs, Fraction(1, 100))

    assert not strict.changed and _counted(strict.domains[0], source_dots.KEPT_UV) >= 1
    assert loose.changed and _counted(loose.domains[0], source_dots.MAX_UV_SLIDE_MILLI_ALPHA) >= 1


def test_a_crease_between_the_domains_is_a_straight_chain_and_its_dots_are_dissolved():
    """Сгиб двух стен (наклон соседа 60 градусов) — прямая цепь: точка на ней ничего не рисует; условия «соседи компланарны» у закона нет."""

    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1, tilt=math.radians(60)))

    inputs, result = _run(one, other)

    assert result.changed and [domain.removed for domain in result.domains] == [{"src:s1", "src:s2", "src:s3"}] * 2
    assert not verify_source_dots(inputs, result, SLIDE)


def test_the_run_of_dissolved_vertices_is_judged_as_a_whole_and_the_chord_depth_is_recorded():
    """Дуга из 11 вершин по 0.09 градуса излома: вся цепочка ушла бы от хорды на 19 мм, растворяется столько, сколько вмещает бюджет 5 мм."""

    xs = [float(i) for i in range(11)]
    arc = {i: (i * i) / (2 * 637.0) for i in range(11)}
    batch = _sound(_strip("a", xs, sag=arc))
    inputs = [_input(0, batch)]
    loose = Fraction(1, 10)  # наклон хорды сдвигает долю по проекции: UV здесь не предмет, предмет — глубина хорды

    result = reconcile_source_dots(inputs, loose)

    (domain,) = result.domains
    assert 0 < len(domain.removed) < 9
    assert _counted(domain, source_dots.KEPT_CHORD) >= 1
    assert 0 < _counted(domain, source_dots.MAX_CHORD_NM) <= 5_000_000
    assert not verify_source_dots(inputs, result, loose)


def test_the_answer_does_not_depend_on_the_order_of_the_domains():
    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1, rung=2))
    forward = _run(one, other)[1]
    inputs = [_input(1, other), _input(0, one)]
    backward = reconcile_source_dots(inputs, SLIDE)

    assert {domain.key: (domain.removed, domain.counters) for domain in forward.domains} == {
        domain.key: (domain.removed, domain.counters) for domain in backward.domains
    }


def test_a_run_without_dots_is_the_input_itself():
    batch = _sound(_strip("a", [0.0, 1.0]))

    inputs, result = _run(batch)

    assert not result.changed and result.domains[0].batch is batch and not result.domains[0].counters


def _forced(monkeypatch, *, bend=None, chord=None, slide=SLIDE, batches=None):
    """Проход под ослабленным законом (бракованный итог), а проверка — под настоящим: красный контроль независимого пересчёта."""

    batches = batches or (_sound(_strip("a", XS)),)
    inputs = [_input(number, batch) for number, batch in enumerate(batches)]
    with monkeypatch.context() as patch:
        if bend is not None:
            patch.setattr(source_dots, "BEND_LIMIT", bend)
        if chord is not None:
            patch.setattr(source_dots, "CHORD_BUDGET", chord)
        result = reconcile_source_dots(inputs, slide)
    return inputs, result


def test_verification_catches_a_corner_dissolved_by_a_looser_bend_limit(monkeypatch):
    inputs, result = _forced(monkeypatch, bend=10.0, slide=Fraction(1, 2), batches=(_sound(_strip("a", XS, bent=(2,))),))

    assert result.domains[0].removed  # соседи изломанной вершины (излом 2.3 градуса) растворены
    assert "BEND_BEYOND_THE_LIMIT" in verify_source_dots(inputs, result, SLIDE)


def test_verification_catches_a_chord_and_a_slide_beyond_the_recorded_maximum(monkeypatch):
    xs = [float(i) for i in range(11)]
    arc = {i: (i * i) / (2 * 637.0) for i in range(11)}
    inputs, result = _forced(monkeypatch, chord=Fraction(1, 10), slide=Fraction(1, 2), batches=(_sound(_strip("a", xs, sag=arc)),))
    assert len(result.domains[0].removed) == 9
    assert "CHORD_DEPTH_BEYOND_RECORDED_MAXIMUM" in verify_source_dots(inputs, result, SLIDE)

    inputs, result = _forced(monkeypatch, slide=Fraction(1, 2), batches=(_sound(_strip("a", XS, skew={2: 0.3})),))
    assert result.changed
    assert "UV_SLIDE_BEYOND_RECORDED_MAXIMUM" in verify_source_dots(inputs, result, SLIDE)


def test_verification_catches_a_place_dissolved_in_one_domain_only_and_a_stale_digest():
    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1))
    inputs, result = _run(one, other)
    only_first = dataclasses.replace(
        result,
        domains=(result.domains[0], dataclasses.replace(result.domains[1], batch=other, removed=frozenset(), content_digest="")),
    )
    stale = dataclasses.replace(result, domains=(dataclasses.replace(result.domains[0], content_digest="0" * 64), result.domains[1]))

    assert "PLACE_DISSOLVED_IN_SOME_DOMAINS_ONLY" in verify_source_dots(inputs, only_first, SLIDE)
    assert "CONTENT_DIGEST_IS_STALE" in verify_source_dots(inputs, stale, SLIDE)


def test_a_failed_verification_dissolves_nothing_and_names_the_skip(monkeypatch):
    one, other = _sound(_strip("a", XS)), _sound(_strip("b", XS, side=-1))
    inputs = [_input(0, one), _input(1, other)]
    monkeypatch.setattr(source_dots, "verify_source_dots", lambda *_args: ("BATCH_DOES_NOT_VALIDATE",))

    result = reconcile_source_dots(inputs, SLIDE)

    assert not result.changed and result.problems == ("BATCH_DOES_NOT_VALIDATE",)
    assert all(
        domain.batch is batch and dict(domain.counters) == {source_dots.SKIPPED_UNVERIFIED: 1}
        for domain, batch in zip(result.domains, (one, other))
    )


# ---------------------------------------------------------------------------
# Поле: настоящие батчи `sagging_wall` под законом `SILHOUETTE_TOPOLOGY_V1`
# ---------------------------------------------------------------------------

FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"


def _field_batch(folder, request_file="decal_request.json", alpha=None):
    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.ids import PolicyId
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.domain import materialize_domain
    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

    root = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    text = (root / request_file).read_text(encoding="utf-8")
    if alpha:
        text = text.replace('"value":"0.6"', f'"value":"{alpha}"')
    request = kernel.DecalRequestCodecV1.loads(text.encode("utf-8"))
    prepared = prepare_conveyor(snapshot, request)
    result = materialize_domain(
        prepared,
        conveyor_coverage(prepared, request.requested_alpha.value),
        request=materialization_request(prepared, uv_policy_id=PolicyId("UV_DIRECT_STRIP_V1")),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1,
    )
    assert result.is_materialized
    return result


@pytest.mark.parametrize(
    "case",
    (("sagging_wall_convex_partition_v1", "decal_request.json", None), ("sagging_wall_rung_chord_v1", "decal_request_alpha_0.6.json", "0.987")),
)
def test_the_field_domains_pass_the_law_and_the_verification_and_the_dissolved_ones_leave_a_sound_batch(case):
    result = _field_batch(*case)
    item = SourceDotInputV1(0, result.batch, (0.0, 0.0, 0.0), tuple(result.vertex_normals))

    found = reconcile_source_dots([item], SLIDE)

    assert not found.problems and not verify_source_dots([item], found, SLIDE)
    (domain,) = found.domains
    assert len(domain.batch.vertices) == len(result.batch.vertices) - len(domain.removed)
    assert not validate_geometry_batch(domain.batch)
    if domain.removed:
        assert dict(domain.overrides)["MATERIALIZE_VERTICES"] == len(domain.batch.vertices)
        assert all(key.startswith("src:") for key in domain.removed)
