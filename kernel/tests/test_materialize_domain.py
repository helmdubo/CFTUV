"""`materialize_domain`: покрытие домена -> валидный `GeometryBatchV1`, именованные отказы.

Три слоя проверки, и каждый на своём числе:

* поле и синтетика (`building_002_*`, цепь из двух и трёх рёбер) идут ПОЛНЫМ
  путём — подготовка, покрытие, материализация, валидатор батча, аудит сетки;
* корпус стенда (`named_corpus`) идёт сборкой от разбиения, без снапшота:
  много форм, дыры, звёзды, и все они обязаны дать валидный батч;
* отказы — по одному на каждое имя исхода, на настоящем домене.
"""

from __future__ import annotations

import dataclasses
import pickle
from decimal import Decimal
from fractions import Fraction
from functools import lru_cache
from types import SimpleNamespace

import pytest

from cftuv_envelope import GeometryBatchCodecV1
from cftuv_envelope.exact_sqrt_sum import reset_factorization_memory
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import admit, assemble, domain, frames
from cftuv_envelope.materialize.admit import MaterializationOutcome, PlanarityKind
from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.numeric import LocalVector3V1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.validation import validate_geometry_batch
from cftuv_envelope.wavefront.conveyor import ConveyorOutcome

import materialize_factories as factories
from wavefront_cases import named_corpus

UV = PolicyId("UV_DIRECT_STRIP_V1")
CASES = (
    "weighted",
    "point_contact",
    "full_selection",
    "two_edge",
    "straight3",
)

#: Золотые дайджесты малых случаев. Меняются ТОЛЬКО осознанно: любое их
#: движение — смена семантики батча (семантический) либо его содержимого
#: (содержательный, он видит триангуляцию).
GOLDEN = {
    "weighted": (
        "a4360476d7143bb595dc2cfc839d394c270a03a57c05faf3173c0d35fd66175d",
        "6c9595622c6553df029c39e7de416eb99668d03507cc359ae209fbbdffba3fcb",
    ),
    "point_contact": (
        "13c9760ddaf4789159392c304703f648a4b48e25fdc4290cf2b0521ff36940be",
        "1df330928f921ffa0274e7345bafe3fedecb5cf12b745a191c503cebc026f90e",
    ),
    "two_edge": (
        "90759ea94fed3a6102a32fa6bda16a85b83da95dab4ff0c3b0c19a6b555580a0",
        "15c1a2b06d84c9a9030ee6e79e189360b79730601183583e35d7d3b0b9c54840",
    ),
    "straight3": (
        "2e9294e7de68f084672b0095c594fcb0da251bc407d5bc3feb1cc13e2f8f43a7",
        "f9988c58176be1d6d0dacdc12aaa86efcfc12116c7d8db26aaf3405e0760e7c1",
    ),
}


@lru_cache(maxsize=None)
def _case(name):
    if name == "weighted":
        return factories.field_domain("building_002_weighted_normals_v1")
    if name == "point_contact":
        return factories.field_domain("building_002_point_contact_v1")
    if name == "full_selection":
        return factories.field_domain("building_002_full_selection_v1")
    if name == "two_edge":
        return factories.two_edge_chain_domain()
    return factories.straight_chain_domain()


def _run(name, **kwargs):
    prepared, coverage, request = _case(name)
    request = dataclasses.replace(request, uv_policy_id=UV)
    return materialize_domain(prepared, coverage, request=request, **kwargs)


def _alpha_decimal(coverage) -> Decimal:
    return Decimal(coverage.alpha.numerator) / Decimal(coverage.alpha.denominator)


# --------------------------------------------------------------------------
# Верный путь
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", CASES)
def test_every_case_materializes_to_a_valid_manifold_batch(name):
    result = _run(name)
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    assert validate_geometry_batch(result.batch) == ()
    prepared = _case(name)[0]
    normal = prepared.context.snapshot.surface_ir.source_faces
    first = sorted(normal, key=lambda item: item.face_id.value)[0].polygon_normal
    audit = audit_batch(result.batch, (first.x, first.y, first.z))
    assert audit.problems() == ()
    # Сетка согласована с исходной гранью, и UV не вывернут нигде, кроме
    # вырожденных (веерных) треугольников — они не «вывернуты», а нулевые.
    assert audit.flipped_vs_source == 0
    assert audit.uv_reversed == 0
    assert 0.0 <= audit.v_min and audit.v_max <= 1.0


@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_golden_semantic_and_content_digests(name):
    result = _run(name)
    semantic, content = GOLDEN[name]
    assert result.batch.semantic_digest.value == semantic
    assert result.content_digest == content


@pytest.mark.parametrize("name", CASES)
def test_the_front_is_exactly_alpha_and_the_source_is_exactly_zero(name):
    """`v = 0` на источнике и `r = alpha`, `v = 1` на внешнем крае — ТОЧНО."""

    result = _run(name)
    batch = result.batch
    alpha = _alpha_decimal(_case(name)[1])
    facts = {
        (fact.semantic_region_id.value, fact.vert_key.value): fact
        for fact in batch.station_facts
    }
    checked_rim = checked_source = 0
    for chain in batch.boundary_chains:
        _tag, kind, region, _number = chain.semantic_boundary_id.value.split(":")
        for key in chain.ordered_vert_keys:
            fact = facts.get((f"region:{region}", key.value))
            if fact is None:
                continue
            if kind == "RIM":
                assert fact.source_r.value == alpha, (name, key)
                checked_rim += 1
            elif kind == "SOURCE":
                assert fact.source_r.value == 0, (name, key)
                checked_source += 1
    assert checked_rim and checked_source
    # UV: на граничных вершинах рима `v` ровно 1.0, на источнике ровно 0.0.
    rim_keys = {
        (region_key, key)
        for chain in batch.boundary_chains
        if ":RIM:" in chain.semantic_boundary_id.value
        for region_key in (
            "region:" + chain.semantic_boundary_id.value.split(":")[2],
        )
        for key in (item.value for item in chain.ordered_vert_keys)
    }
    for face in batch.faces:
        for key, fact in zip(face.ordered_vert_keys, face.uv_facts):
            if (face.semantic_region_id.value, key.value) in rim_keys:
                assert fact.uv.v == 1.0


def test_the_station_accumulates_along_a_two_edge_chain():
    """`s` растёт по ЦЕПИ: v0 -> 0, v1 -> 6, v2 -> 6 * 218453 / 131072."""

    result = _run("two_edge")
    facts = {fact.vert_key.value: fact for fact in result.batch.station_facts}
    assert facts["src:v0"].source_s.value == 0
    assert facts["src:v1"].source_s.value == 6
    assert facts["src:v2"].source_s.value == Decimal(6 * 218453) / Decimal(131072)
    # `u` — та же станция, делённая на alpha = 1: непрерывна вдоль цепи.
    uv = {
        key.value: fact.uv
        for face in result.batch.faces
        for key, fact in zip(face.ordered_vert_keys, face.uv_facts)
    }
    assert uv["src:v1"].u == 6.0
    assert uv["src:v2"].u == float(Fraction(6 * 218453, 131072))
    # Два коллинеарных ребра одной цепи слились в ОДНУ грань и один регион.
    assert dict(result.counters)["MATERIALIZE_SEPARATORS_MERGED"] == 1
    assert len(result.batch.semantic_regions) == 1


def test_a_vertex_fan_is_constant_station_and_its_uv_is_degenerate_by_name():
    result = _run("point_contact")
    counters = dict(result.counters)
    constant = [
        fact
        for fact in result.batch.station_facts
        if fact.station_model_id.value == "CONSTANT_PHYSICAL_ENDPOINT_S"
    ]
    assert constant and counters["MATERIALIZE_STATION_CONSTANT_S"] == len(constant)
    assert counters["MATERIALIZE_FAN_FACES"] == 2
    by_region: dict = {}
    for fact in constant:
        by_region.setdefault(fact.semantic_region_id, set()).add(fact.source_s.value)
    # Станция веера константна во всём его регионе.
    assert all(len(values) == 1 for values in by_region.values())
    # Следствие закона V1, названное числом: треугольники веера вырождены в UV.
    assert counters["MATERIALIZE_TRIANGLES_UV_DEGENERATE"] >= counters[
        "MATERIALIZE_FAN_FACES"
    ]


def test_a_mirrored_domain_materializes_with_consistent_winding():
    """Окно в зеркальную карту: порядок обхода разворачивается по ориентации."""

    prepared, coverage, request = _case("two_edge")
    mirrored = dataclasses.replace(
        prepared.context.frame,
        chart_orientation=type(prepared.context.frame.chart_orientation)(
            "COORDINATE_CW_MATCHES_OWNER_PATCH"
        ),
    )
    swapped = dataclasses.replace(
        prepared, context=dataclasses.replace(prepared.context, frame=mirrored)
    )
    result = materialize_domain(
        swapped, coverage, request=dataclasses.replace(request, uv_policy_id=UV)
    )
    straight = _run("two_edge")
    assert result.outcome is MaterializationOutcome.MATERIALIZED
    # Сетка та же по составу, но каждый треугольник идёт в ОБРАТНОМ порядке.
    left = {tuple(face.ordered_vert_keys) for face in straight.batch.faces}
    right = {tuple(face.ordered_vert_keys[::-1]) for face in result.batch.faces}
    assert len(left) == len(right) == len(straight.batch.faces)
    assert {frozenset(item) for item in left} == {frozenset(item) for item in right}


def test_materialization_is_deterministic_and_the_answer_survives_a_pickle():
    first = _run("full_selection")
    second = _run("full_selection")
    assert first.content_digest == second.content_digest
    assert first.batch == second.batch
    # Цена (бюджет, секунды) — не ответ: она зависит от тепла памяти разложений.
    answer = lambda item: tuple(  # noqa: E731
        pair for pair in item.counters if not pair[0].startswith("EXACT_WORK_")
    )
    assert answer(first) == answer(second)
    restored = pickle.loads(pickle.dumps(first))
    assert restored == first
    assert restored.content_digest == first.content_digest


def test_the_batch_survives_the_codec_round_trip():
    result = _run("point_contact")
    payload = GeometryBatchCodecV1.dumps(result.batch)
    assert GeometryBatchCodecV1.loads(payload) == result.batch
    assert validate_geometry_batch(GeometryBatchCodecV1.loads(payload)) == ()


def test_the_semantic_digest_does_not_see_the_triangulation_and_the_content_does(
    monkeypatch,
):
    """`TessellationDigestEquivalence`: другая диагональ — тот же смысл."""

    base = _run("straight3")
    original = assemble.triangulate_exact

    def rotated(points, budget):
        size = len(points)
        triangles = original(points[1:] + points[:1], budget)
        return None if triangles is None else tuple(
            tuple((index + 1) % size for index in triangle) for triangle in triangles
        )

    monkeypatch.setattr(assemble, "triangulate_exact", rotated)
    other = _run("straight3")
    assert other.outcome is MaterializationOutcome.MATERIALIZED
    assert other.batch.faces != base.batch.faces
    assert other.content_digest != base.content_digest
    assert other.batch.semantic_digest == base.batch.semantic_digest


# --------------------------------------------------------------------------
# Именованные отказы
# --------------------------------------------------------------------------


def test_a_coverage_that_is_not_exact_is_refused_by_name():
    prepared, coverage, request = _case("weighted")
    broken = dataclasses.replace(
        coverage, outcome=ConveyorOutcome.COVERAGE_DID_NOT_CLOSE
    )
    result = materialize_domain(
        prepared, broken, request=dataclasses.replace(request, uv_policy_id=UV)
    )
    assert result.outcome is MaterializationOutcome.COVERAGE_IS_NOT_EXACT
    assert result.batch is None and result.content_digest == ""
    assert "COVERAGE_DID_NOT_CLOSE" in result.detail


def test_a_non_planar_descriptor_is_refused_by_name():
    prepared, coverage, request = _case("weighted")
    curved = dataclasses.replace(
        prepared,
        context=dataclasses.replace(
            prepared.context, frame=SimpleNamespace(kind="SURFACE_METRIC_V2")
        ),
    )
    result = materialize_domain(
        curved, coverage, request=dataclasses.replace(request, uv_policy_id=UV)
    )
    assert result.outcome is MaterializationOutcome.DOMAIN_IS_NOT_PLANAR_ADMITTED
    assert result.detail == "SimpleNamespace"


def test_an_unsupported_uv_policy_is_refused_by_name_and_the_debug_one_is_among_them():
    prepared, coverage, request = _case("weighted")
    for name in ("ENVELOPE_DEBUG_NO_UV_V1", "SOME_FUTURE_ATLAS_V2"):
        result = materialize_domain(
            prepared,
            coverage,
            request=dataclasses.replace(request, uv_policy_id=PolicyId(name)),
        )
        assert result.outcome is MaterializationOutcome.UV_POLICY_UNSUPPORTED
        assert result.detail == name


def test_an_owner_without_a_loop_edge_leaves_its_station_chain_unnamed(monkeypatch):
    original = domain.chain_station_table

    def without_edges(prepared, budget):
        return dataclasses.replace(original(prepared, budget), edges={})

    monkeypatch.setattr(domain, "chain_station_table", without_edges)
    result = _run("weighted")
    assert result.outcome is MaterializationOutcome.STATION_CHAIN_UNNAMED
    assert "no chain-use edge" in result.detail


def test_a_face_without_any_name_for_its_claim_is_refused():
    face = SimpleNamespace(
        envelope_instance_id=None,
        envelope_spec_id="",
        source_chain_id=None,
        owner=(0, 0, 1, 0),
        merged_owners=(),
    )
    with pytest.raises(frames.MaterializationRefusal) as refusal:
        frames.resolve_frame(SimpleNamespace(), "r", face, None, frozenset())
    assert refusal.value.outcome is MaterializationOutcome.STATION_CHAIN_UNNAMED


def test_merged_owners_spanning_two_station_runs_make_an_ambiguous_frame():
    edge = lambda run: SimpleNamespace(run_id=run)  # noqa: E731
    table = SimpleNamespace(
        edge_of_owner=lambda region, owner: edge("a" if owner[0] == 0 else "b"),
        runs={},
    )
    face = SimpleNamespace(
        owner=(0, 0, 1, 0), merged_owners=((2, 0, 3, 0),)
    )
    with pytest.raises(frames.MaterializationRefusal) as refusal:
        frames._strip_frame(table, "r", face)
    assert refusal.value.outcome is MaterializationOutcome.STATION_FRAME_IS_AMBIGUOUS


def test_a_contour_that_does_not_tessellate_is_refused_by_name(monkeypatch):
    monkeypatch.setattr(assemble, "triangulate_exact", lambda points, budget: None)
    result = _run("weighted")
    assert result.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert result.batch is None


def test_a_wrong_triangulation_is_caught_by_the_exact_area_check(monkeypatch):
    """Сумма площадей треугольников обязана равняться площади грани ТОЧНО."""

    def fan(points, budget):
        return tuple((0, index, index + 1) for index in range(1, len(points) - 2))

    monkeypatch.setattr(assemble, "triangulate_exact", fan)
    result = _run("weighted")
    assert result.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert "areas differ" in result.detail


def test_a_batch_the_validator_rejects_is_named_with_its_issues(monkeypatch):
    issue = SimpleNamespace(code=SimpleNamespace(value="GEOMETRY_BATCH"), path=("faces",))
    monkeypatch.setattr(domain, "validate_geometry_batch", lambda batch: (issue,))
    result = _run("weighted")
    assert result.outcome is MaterializationOutcome.BATCH_DID_NOT_VALIDATE
    assert "GEOMETRY_BATCH@faces" in result.detail
    assert result.batch is None


def test_a_mesh_the_audit_rejects_is_named_with_its_problem(monkeypatch):
    real = domain.audit_batch

    def cracked(batch, normal):
        return dataclasses.replace(real(batch, normal), boundary_chain_mismatch=2)

    monkeypatch.setattr(domain, "audit_batch", cracked)
    result = _run("weighted")
    assert result.outcome is MaterializationOutcome.BATCH_DID_NOT_VALIDATE
    assert result.detail == "AUDIT:BOUNDARY_DOES_NOT_MATCH_CHAINS"


def test_an_exhausted_budget_is_a_named_outcome_not_a_hang():
    prepared, coverage, request = _case("weighted")
    reset_factorization_memory()
    result = materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        work_budget=factories.budget(cap=0),
    )
    assert result.outcome is MaterializationOutcome.EXACT_WORK_BUDGET_EXHAUSTED
    assert "EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED" in result.detail
    # Исход несёт счёт работы: отказ без числа не отличить от зависания.
    assert dict(result.counters)["EXACT_WORK_SPENT"] > 0


# --------------------------------------------------------------------------
# Допуск и диагностики
# --------------------------------------------------------------------------


def _near_planar_certificate():
    from cftuv_envelope.contracts.analysis import SourceVertexV1
    from cftuv_envelope.contracts.surface import SourceFaceV1
    from cftuv_envelope.ids import (
        PatchDomainId,
        PatchId,
        PhysicalChainId,
        SourceFaceId,
        SourceRevision,
        SourceVertexId,
    )
    from cftuv_envelope.numeric import LocalPoint3V1
    from cftuv_envelope.planar_metric import build_rational_affine_planar_metric
    from cftuv_envelope.contracts.metric import PlanarityAdmissionLawV1

    ids = [SourceVertexId(f"v{index}") for index in range(4)]
    positions = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 1e-12))
    vertices = [
        SourceVertexV1(vertex_id=item, position=LocalPoint3V1(*position))
        for item, position in zip(ids, positions, strict=True)
    ]
    face = SourceFaceV1(
        face_id=SourceFaceId("f0"),
        patch_id=PatchId("p0"),
        vertex_cycle=tuple(ids),
        edge_cycle=tuple(PhysicalChainId(f"e{index}") for index in range(4)),
        polygon_normal=LocalPoint3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )
    return build_rational_affine_planar_metric(
        source_revision=SourceRevision("rev"),
        patch_domain_id=PatchDomainId("d0"),
        owner_patch_id=PatchId("p0"),
        source_vertices=vertices,
        source_faces=[face],
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
    ).planarity_certificate


def test_a_near_planar_domain_is_admitted_and_the_lift_is_named():
    prepared, coverage, request = _case("weighted")
    certificate = _near_planar_certificate()
    near = dataclasses.replace(prepared.context.frame, planarity_certificate=certificate)
    swapped = dataclasses.replace(
        prepared, context=dataclasses.replace(prepared.context, frame=near)
    )
    request = dataclasses.replace(request, uv_policy_id=UV)
    admission = admit.admit_domain(swapped, coverage, request)
    assert admission.outcome is None
    assert admission.planarity is PlanarityKind.NEAR_PLANAR
    lines: list = []
    found = domain._diagnostics(
        swapped, SimpleNamespace(restart_chain_ids=frozenset()), admission.planarity, lines
    )
    assert [item.outcome for item in found] == [
        NamedOutcome.NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE
    ]
    assert "residual_budget=" in lines[0]


def test_a_restart_and_a_degraded_miter_are_named_in_the_batch():
    prepared, _coverage, _request = _case("weighted")
    corner = SimpleNamespace(corner_relation_id="corner:1", reason="ANCHOR_IS_NOT_RATIONAL")
    stub = SimpleNamespace(
        regions=(SimpleNamespace(degraded_miter_corners=(corner,)),),
        context=prepared.context,
    )
    lines: list = []
    found = domain._diagnostics(
        stub,
        SimpleNamespace(restart_chain_ids=frozenset({"chain:a"})),
        PlanarityKind.PLANAR_EXACT,
        lines,
    )
    assert {item.outcome for item in found} == {
        NamedOutcome.U_RESTARTS_AT_DOMAIN_BORDER,
        NamedOutcome.DEGRADED_MITER_CORNER_IN_GEOMETRY,
    }


# --------------------------------------------------------------------------
# Корпус стенда
# --------------------------------------------------------------------------

#: Корпус стенда целиком: 22 формы на двух alpha. Разбиение каждой доходит до
#: `EXACT`, и число заморожено: молчаливое выпадение формы ловится здесь.
CORPUS_ASSEMBLED = 44


@lru_cache(maxsize=1)
def _corpus_results():
    rows = []
    for name, polygon in named_corpus():
        for alpha in (Fraction(1), Fraction(3, 2)):
            built = factories.assemble_polygon_batch(polygon, alpha)
            rows.append((name, alpha, built))
    return tuple(rows)


def test_the_corpus_assembles_valid_batches_with_a_sound_mesh():
    assembled = 0
    for name, alpha, built in _corpus_results():
        assert built is not None, (name, alpha)
        batch, frame_faces = built
        assembled += 1
        assert validate_geometry_batch(batch) == (), (name, alpha)
        audit = audit_batch(batch, (0.0, 0.0, 1.0))
        assert audit.problems() == (), (name, alpha, audit)
        assert audit.flipped_vs_source == 0, (name, alpha)
        # Площадь меша равна точной площади покрытия: ни дыр, ни наложений.
        mesh = Fraction(0)
        position = {item.vert_key: item.position for item in batch.vertices}
        for face in batch.faces:
            a, b, c = (position[key] for key in face.ordered_vert_keys)
            mesh += Fraction((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
        exact = Fraction(0)
        for item in frame_faces:
            low, high = item.face.doubled_area.enclosure(64)
            exact += (low + high) / 2
        assert abs(mesh - exact) <= Fraction(1, 10**6) * max(Fraction(1), exact)
    assert assembled == CORPUS_ASSEMBLED


def test_the_corpus_digests_are_stable_between_two_assemblies():
    first = _corpus_results()
    second = tuple(
        (name, alpha, factories.assemble_polygon_batch(polygon, alpha))
        for (name, polygon), alpha in (
            ((name, polygon), alpha)
            for name, polygon in named_corpus()
            for alpha in (Fraction(1), Fraction(3, 2))
        )
    )
    for (name, alpha, left), (_n, _a, right) in zip(first, second):
        assert left[0].semantic_digest == right[0].semantic_digest, (name, alpha)
        assert left[0] == right[0], (name, alpha)
