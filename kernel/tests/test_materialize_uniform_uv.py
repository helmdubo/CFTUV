"""Точные UV-формы полного семантического графа; фазовый план задаётся явно."""

from __future__ import annotations

import dataclasses
from decimal import Decimal
from fractions import Fraction

import pytest

import materialize_factories as fixtures
from cftuv_envelope import backend
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1 as S
from cftuv_envelope.materialize import assemble, domain, uv_uniform as uv
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.parametric import ParametricRequestV1
from cftuv_envelope.materialize.lift import sqrt_sum_binary64 as number
from cftuv_envelope.materialize.stations import chain_station_table, station_of
from cftuv_envelope.materialize.uv_phase import (
    UniformUvPhaseFactsV1, UniformUvPhaseLinkV1, collect_phase_facts,
    compile_uniform_phase_plan, compile_uniform_uv_context,
)


def _single_patch_phases(prepared, table, budget):
    """Только фикстура: согласовать фазы одного полного графа, без объявления межпатчевой приёмки."""

    _owners, nodes, joins = uv.uniform_support_graph(prepared, table, budget)
    roots = {}
    for key, support in sorted(nodes.items()):
        if support.anchor is not None:
            roots.setdefault(key[0], key)
    free = {key: dataclasses.replace(support, anchor=support.anchor if roots.get(key[0]) == key else None)
            for key, support in nodes.items()}
    values, _components, conflict = uv._walk(free, joins, set())
    assert conflict is None, f"fixture requires a declared cycle cut: {conflict[0]}"
    phases = {}
    for key, support in nodes.items():
        if support.anchor is not None:
            chain = table.runs[key[-1]].chain_id
            phase = (values[key][0]-support.anchor).scaled(Fraction(1, table.scale))
            assert chain not in phases or phases[chain] == phase
            phases[chain] = phase
    return phases


def _point_contact(alpha="0.25"):
    return fixtures.field_domain("building_002_point_contact_v1", alpha=alpha)


def test_the_full_support_plan_has_four_continuous_fan_joins_and_preserves_shifted_source_phases():
    prepared, _coverage, _request = _point_contact()
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    phases = _single_patch_phases(prepared, table, budget)
    plan = uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres=phases)
    assert len(plan.joins) == 4 and plan.cuts == ()
    for (rid, owner), chart in plan.charts.items():
        if len(owner) == 4:
            edge = table.edge_of_owner(rid, owner)
            run = table.runs[edge.run_id]
            for point in (owner[:2], owner[2:]):
                exact = uv._point(point)
                assert chart.value(exact, uv.ZERO) == station_of(run, exact) + phases[run.chain_id].scaled(table.scale)


def test_missing_or_uncoordinated_phases_are_named_and_never_insert_a_fan_seam():
    prepared, _coverage, _request = _point_contact()
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    with pytest.raises(uv.UniformUvRefusal, match="UV_UNIFORM_PHASE_PLAN_MISSING"):
        uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres={})
    with pytest.raises(uv.UniformUvRefusal, match="UV_UNIFORM_PHASE_PLAN_INCONSISTENT"):
        uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres={run.chain_id: uv.ZERO for run in table.runs.values()})


def test_all_support_charts_have_exact_unit_metric_area_and_stable_coefficients_when_reordered():
    prepared, _coverage, _request = _point_contact()
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    phases = _single_patch_phases(prepared, table, budget)
    plan = uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres=phases)
    g, h, k = table.gram
    for region in prepared.regions:
        for face in region.partition.faces:
            chart = plan.charts[(region.region_id, tuple(face.owner))]
            inverse = S.radical(1, 1/Fraction(face.line.q), budget)
            rx, ry = inverse.scaled(face.line.a), inverse.scaled(face.line.b)
            determinant = chart.tangent[0]*ry-chart.tangent[1]*rx
            assert determinant*determinant == S.rational(g*k-h*h)
    shuffled = dataclasses.replace(prepared, regions=tuple(
        dataclasses.replace(region, partition=dataclasses.replace(region.partition, faces=tuple(reversed(region.partition.faces))))
        for region in reversed(prepared.regions)
    ))
    other = uv.build_uniform_plan(shuffled, table, budget, phase_offsets_metres=phases)
    assert plan == other


def test_current_alpha_never_changes_the_full_support_gauge_or_affine_forms():
    prepared, _coverage, _request = _point_contact("0.249")
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    phases = _single_patch_phases(prepared, table, budget)
    first = uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres=phases)
    for alpha in ("0.25", "0.251", "0.5"):
        other, _coverage, _request = _point_contact(alpha)
        actual = uv.build_uniform_plan(other, chain_station_table(other, budget), budget, phase_offsets_metres=phases)
        assert actual == first


def _mapped_batch(prepared, coverage, monkeypatch, phases):
    capture = {}
    original = domain._shaped

    def shaped(*args, **kwargs):
        capture["points"] = args[6][0]
        return original(*args, **kwargs)

    def assembled(oracle, **kwargs):
        capture["arguments"] = kwargs
        return oracle(**kwargs)

    with monkeypatch.context() as patch:
        patch.setattr(domain, "_shaped", shaped)
        patch.setattr(backend, "assemble_compute", assembled)
        direct = domain.materialize_domain(prepared, coverage, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"))
    assert direct.is_materialized, direct.detail
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    plan = uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres=phases)
    args = capture["arguments"]
    values, forms = uv.map_uniform_uv(plan, args["frame_faces"], args["layout"], args["polygons"], capture["points"], args["facts"],
                                     source_loci={slot: capture["points"][slot[1]] for slot, pair in args["facts"].items() if slot[1].startswith("src:") and pair[1].is_zero})
    batch = assemble.assemble_batch(**{**args, "uv_numerators": values})
    return direct.batch, batch, values, forms, capture, table


def test_the_uv_map_preserves_physical_station_records_and_gives_positive_unit_metric_area(monkeypatch):
    prepared, coverage, _request = _point_contact()
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    phases = _single_patch_phases(prepared, table, budget)
    direct, batch, values, _forms, capture, table = _mapped_batch(prepared, coverage, monkeypatch, phases)
    assert batch.vertices == direct.vertices and batch.station_facts == direct.station_facts
    assert len(batch.faces) == len(direct.faces) == 8
    g, h, k = table.gram
    metric_area = S.radical(1, g*k-h*h, budget)
    points = capture["points"]
    for face in batch.faces:
        region = int(face.semantic_region_id.value.split(":")[1])
        keys = [key.value for key in face.ordered_vert_keys]
        for index in range(1, len(keys)-1):
            a, b, c = keys[0], keys[index], keys[index+1]
            n0, n1, n2 = values[(region, a)], values[(region, b)], values[(region, c)]
            uv_area = (n1[0]-n0[0])*(n2[1]-n0[1])-(n1[1]-n0[1])*(n2[0]-n0[0])
            p0, p1, p2 = points[a], points[b], points[c]
            chart_area = (p1[0]-p0[0])*(p2[1]-p0[1])-(p1[1]-p0[1])*(p2[0]-p0[0])
            assert uv_area.sign(budget=budget) > 0
            assert uv_area == chart_area*metric_area


def test_final_uv_numerators_are_exactly_affine_across_widths(monkeypatch):
    prepared, _coverage, _request = _point_contact()
    budget = fixtures.budget()
    phases = _single_patch_phases(prepared, chain_station_table(prepared, budget), budget)
    values = []
    for alpha in ("0.249", "0.25", "0.251"):
        prepared, coverage, _request = _point_contact(alpha)
        mapped = _mapped_batch(prepared, coverage, monkeypatch, phases)
        current, args = mapped[2], mapped[4]["arguments"]
        values.append({(frame.face.owner, key): current[(args["layout"].region_of(frame), key)]
                       for frame, rings in zip(args["frame_faces"], args["polygons"])
                       for ring in rings for key in ring})
    assert set(values[0]) == set(values[1]) == set(values[2])
    assert len(values[0]) == 18
    for slot in values[0]:
        assert all((values[0][slot][axis]+values[2][slot][axis]-values[1][slot][axis].scaled(2)).is_zero for axis in (0, 1))


def test_the_request_phase_compiler_uses_existing_endpoint_stations_and_keeps_all_four_fan_joins():
    prepared, _coverage, _request = _point_contact()
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    facts = collect_phase_facts(prepared, table, budget)
    context = compile_uniform_uv_context((prepared,))
    assert len(facts.chains) == 3 and len(facts.links) == 2
    assert len(context.offsets_metres) == 3 and context.cut_ids == ()
    assert context.offsets_metres[0][1] == uv.ZERO
    plan = uv.plan_from_context(prepared, table, budget, context)
    assert len(plan.joins) == 4 and plan.cuts == ()
    assert context == compile_uniform_uv_context((prepared, prepared))
    with pytest.raises(dataclasses.FrozenInstanceError):
        context.digest = "changed"
    altered = dataclasses.replace(table, scale=table.scale*2)
    with pytest.raises(uv.UniformUvRefusal, match="UV_UNIFORM_PHASE_BASIS_MISMATCH"):
        uv.plan_from_context(prepared, altered, budget, context)


def test_global_phase_cycles_keep_only_proven_old_cuts_and_are_order_independent():
    def link(index, left, right, delta, *, fan=False, old=True):
        return UniformUvPhaseLinkV1(("patch", 0, index, 0), left, right, S.rational(delta), fan, old)

    links = (link(0, "a", "b", 2, fan=True), link(1, "b", "c", 3, old=False), link(2, "a", "c", 7))
    facts = UniformUvPhaseFactsV1(("patch", "basis"), ("a", "b", "c"), links)
    context = compile_uniform_phase_plan((facts,))
    assert context.offsets_metres == (("a", S.rational(0)), ("b", S.rational(2)), ("c", S.rational(5)))
    assert context.cut_ids == (links[2].key,)
    assert context == compile_uniform_phase_plan((dataclasses.replace(facts, chains=tuple(reversed(facts.chains)), links=tuple(reversed(links))),))
    impossible = dataclasses.replace(facts, links=tuple(dataclasses.replace(item, old_seam=False) for item in links))
    with pytest.raises(uv.UniformUvRefusal, match="UV_UNIFORM_PHASE_CYCLE_WITHOUT_OLD_CUT"):
        compile_uniform_phase_plan((impossible,))


@pytest.mark.parametrize("factory,joins,cuts", ((fixtures.two_edge_chain_domain, 0, 0), (fixtures.l_chains_domain, 1, 0), (fixtures.ring_domain, 3, 1)))
def test_full_static_phase_plans_support_straight_l_and_ring_sources(factory, joins, cuts):
    prepared, _coverage, _request = factory()
    budget = fixtures.budget()
    table = chain_station_table(prepared, budget)
    context = compile_uniform_uv_context((prepared,))
    plan = uv.plan_from_context(prepared, table, budget, context)
    assert plan.charts
    assert len(plan.joins) == joins and len(plan.cuts) == cuts


def test_phase_context_is_independent_of_true_requested_width_and_request_identity():
    snapshot, request = fixtures.load_fixture("building_002_point_contact_v1")
    prepared, _coverage = fixtures.prepare_and_cover(snapshot, request)
    context = compile_uniform_uv_context((prepared,))
    for alpha in ("0.125", "0.25", "0.5"):
        changed = dataclasses.replace(request, requested_alpha=dataclasses.replace(request.requested_alpha, value=Decimal(alpha)),
                                      decal_request_id=type(request.decal_request_id)("different-request"))
        other, _coverage = fixtures.prepare_and_cover(snapshot, changed)
        assert compile_uniform_uv_context((other,)) == context


def test_uniform_materialization_is_explicit_and_preserves_direct_geometry_and_physical_stations():
    prepared, coverage, _request = _point_contact()
    context = compile_uniform_uv_context((prepared,))
    direct_request = materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")
    uniform_request = materialization_request(prepared, uv_policy_id="UV_UNIFORM_TRIM_FAN_V1")
    direct = domain.materialize_domain(prepared, coverage, request=direct_request)
    uniform = domain.materialize_domain(prepared, coverage, request=uniform_request, uv_context=context)
    assert uniform.is_materialized, uniform.detail
    assert uniform.batch.vertices == direct.batch.vertices
    assert uniform.batch.station_facts == direct.batch.station_facts
    assert [face.ordered_vert_keys for face in uniform.batch.faces] == [face.ordered_vert_keys for face in direct.batch.faces]
    assert any(face.uv_facts != before.uv_facts for face, before in zip(uniform.batch.faces, direct.batch.faces))
    assert "cftuv.envelope.uv_policy.UV_UNIFORM_TRIM_FAN_V1" in {item.value for item in uniform.batch.contract_versions}
    missing = domain.materialize_domain(prepared, coverage, request=uniform_request)
    assert missing.outcome.value == "UV_UNIFORM_PHASE_PLAN_MISSING"
    altered = dataclasses.replace(context, basis=())
    mismatch = domain.materialize_domain(prepared, coverage, request=uniform_request, uv_context=altered)
    assert mismatch.outcome.value == "UV_UNIFORM_PHASE_BASIS_MISMATCH"


def test_phase_exclusions_are_part_of_identity_and_runtime_work_has_a_bounded_owner():
    prepared, _coverage, _request = _point_contact()
    base = compile_uniform_uv_context((prepared,))
    excluded = compile_uniform_uv_context((prepared,), excluded_domain_ids=(("refused-domain", "STATIC_REFUSAL"),))
    assert excluded.digest != base.digest and ("excluded:refused-domain", "STATIC_REFUSAL") in excluded.basis
    assert base.work_counters


def test_width_event_withdraws_only_the_physical_join_and_keeps_the_static_uv_basis():
    from test_corner_join import EVENT_POLYGON, EVENT_ROUTE, _piece_chain_snapshot
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1

    snapshot, request = _piece_chain_snapshot(EVENT_POLYGON, EVENT_ROUTE, alpha="1")
    prepared, _coverage = fixtures.prepare_and_cover(snapshot, request)
    context = compile_uniform_uv_context((prepared,))
    from cftuv_envelope.wavefront import conveyor_coverage
    for alpha in ("1", "2"):
        coverage = conveyor_coverage(prepared, alpha)
        direct = domain.materialize_domain(prepared, coverage, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
                                           decal_topology_law=DecalTopologyLawV1.TRIANGLES_V1)
        uniform = domain.materialize_domain(prepared, coverage, request=materialization_request(prepared, uv_policy_id="UV_UNIFORM_TRIM_FAN_V1"),
                                            decal_topology_law=DecalTopologyLawV1.TRIANGLES_V1, uv_context=context)
        assert direct.is_materialized and uniform.is_materialized, (alpha, uniform.detail)
        assert uniform.batch.vertices == direct.batch.vertices and uniform.batch.station_facts == direct.batch.station_facts


def test_source_loci_keep_exact_fan_phase_after_physical_source_snap_and_certify_constant_source_uv(monkeypatch):
    prepared, coverage, _request = _point_contact()
    budget = fixtures.budget()
    phases = _single_patch_phases(prepared, chain_station_table(prepared, budget), budget)
    _direct, _batch, values, _forms, capture, table = _mapped_batch(prepared, coverage, monkeypatch, phases)
    plan = uv.build_uniform_plan(prepared, table, budget, phase_offsets_metres=phases)
    args, points = capture["arguments"], capture["points"]
    source_loci = {slot: points[slot[1]] for slot, pair in args["facts"].items() if slot[1].startswith("src:") and pair[1].is_zero}
    assert source_loci
    source_keys = {slot[1] for slot in source_loci}
    snapped = {key: (point[0]+S.rational(1), point[1]-S.rational(1)) if key in source_keys else point for key, point in points.items()}
    mapped, forms = uv.map_uniform_uv(plan, args["frame_faces"], args["layout"], args["polygons"], snapped, args["facts"], source_loci=source_loci)
    assert mapped == values
    slots = {slot for slot in forms if slot in source_loci}
    assert slots
    slopes = uv.uniform_uv_slopes(slots, forms, {key: (S.rational(3), S.rational(5)) for key in source_keys},
                                  {slot: (uv.ZERO, uv.ZERO) for slot in slots}, {}, {})
    assert all(pair == (uv.ZERO, uv.ZERO) for pair in slopes.values())
    with pytest.raises(uv.UniformUvRefusal, match="UV_UNIFORM_SOURCE_LOCUS_MISSING"):
        uv.map_uniform_uv(plan, args["frame_faces"], args["layout"], args["polygons"], snapped, args["facts"], source_loci={})


def test_source_snap_with_new_clip_nodes_names_the_nonaffine_certificate_and_leaves_direct_alone(monkeypatch):
    from types import SimpleNamespace
    from cftuv_envelope.materialize import parametric

    before = {"src:source": (uv.ZERO, uv.ZERO)}
    after = {"src:source": (S.rational(1), uv.ZERO)}
    inputs = SimpleNamespace(uv_numerators={}, cut=SimpleNamespace(points={"new-clip-node": after["src:source"]}),
                             points_before=before, points_after=after, facts_before={(0, "src:source"): (uv.ZERO, uv.ZERO)})
    def unexpected(*args, **kwargs):
        pytest.fail("a nonaffine source clip entered the certificate backend")
    monkeypatch.setattr(backend, "key_slopes_compute", unexpected)
    monkeypatch.setattr(backend, "certificate_core_compute", unexpected)
    with pytest.raises(parametric.ParametricRefusal, match="PARAM_UV_SOURCE_LOCUS_CLIPPED_NONAFFINE"):
        parametric._certificate(inputs, None, None, None, None, None)
    assert parametric._uniform_source_clip_nonaffine(inputs)
    inputs.uv_numerators = None
    assert not parametric._uniform_source_clip_nonaffine(inputs)
    inputs.uv_numerators, inputs.facts_before = {}, {(0, "src:source"): (uv.ZERO, S.rational(1))}
    assert not parametric._uniform_source_clip_nonaffine(inputs)


@pytest.mark.parametrize("name", ("building_002_point_contact_v1", "building_002_full_selection_v1", "mesh2_patch0_cut_fans_v1"))
def test_uniform_silhouette_validates_actual_uv_continuity_and_exact_certificate_loops(name):
    import parametric_theorem as theorem
    from test_parametric_certificate import HOST_LAWS
    from cftuv_envelope.materialize.step import step_domain

    prepared, _coverage, _request = fixtures.field_domain(name)
    context = compile_uniform_uv_context((prepared,))
    request = materialization_request(prepared, uv_policy_id="UV_UNIFORM_TRIM_FAN_V1")
    def run(alpha):
        return step_domain(prepared, alpha, request=request, near_planar_lift_law=HOST_LAWS[0], decal_topology_law=HOST_LAWS[1],
                           digests=False, parametric=True, parametric_exact=True, uv_context=context)
    base = run("0.25")
    assert base.result.is_materialized, base.result.detail
    certificate = base.result.parametric
    assert certificate is not None and theorem.usable(certificate), certificate and certificate.markers
    other = run("0.25025")
    outcome, differences = theorem.theorem_differences(base, other, Fraction("0.25"), Fraction("0.25025"))
    assert outcome == theorem.JUDGED and not differences, (outcome, differences)
