"""DEVELOPABLE (S1), правка аудита: объявленные ПРЯМЫМИ цепи на карте развёртки.

Очередь требует от объявленной прямой цепи ТОЧНОЙ коллинеарности её вершин в карте. Карта
развёртки её не давала: независимая привязка каждой вершины к решётке рвёт любую прямую вне
осей решётки, а внутренняя геометрия патча у вершин цепи может быть не плоской. Теперь
внутренности такой цепи кладутся на хорду между концами (коллинеарность — по построению),
а цепь, которую карта не терпит прямой, — именованный отказ МЕТРИКИ с цепью, вершиной и
оболочкой угла против `π`, а не `SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR` очереди.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope._stretch import stretch_violations
from cftuv_envelope.contracts.metric import (
    DevelopableStraightChainLawV1,
    DevelopableUnfoldCertificateV1,
    ExactPoint2V1,
    ExactRationalV1,
    NearPlanarProjectionCertificateV1,
)
from cftuv_envelope.declared_chains import declared_straight_chain_vertices
from cftuv_envelope.ids import PatchDomainId, PhysicalChainId, SourceVertexId
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.reference.evaluation_geometry import (
    SourceDeclaredStraightChainIsNotLinear,
    _base_nodes,
    _chain_info,
    _source_coordinates,
    chart_lattice_for_frame,
)
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
)

import developable_factories as factories
from developable_factories import DOMAIN, PATCH, REVISION
from developable_route import build_metric


def _chart(parts, chain=None, **overrides):
    chains = () if chain is None else (chain,)
    return factories.developable_chart(parts, declared_straight_chains=chains, **overrides)


def _coordinates(nodes, chain):
    return [(Fraction(nodes[vertex][0]), Fraction(nodes[vertex][1])) for vertex in chain]


def _on_one_line_in_order(points) -> bool:
    first, last = points[0], points[-1]
    span = (last[0] - first[0], last[1] - first[1])
    reach = span[0] * span[0] + span[1] * span[1]
    previous = Fraction(0)
    for point in points[1:-1]:
        offset = (point[0] - first[0], point[1] - first[1])
        along = offset[0] * span[0] + offset[1] * span[1]
        if span[0] * offset[1] - span[1] * offset[0] or not previous < along < reach:
            return False
        previous = along
    return bool(reach)


def _queue_predicate(record, chain):
    """Предикат очереди ТОЧНО: `_chain_info` на координатах метрики; исключение — отказ очереди."""

    metric = record.metric
    stub = SimpleNamespace(ordered_source_vertex_ids=tuple(chain))
    return _chain_info(
        stub,
        _source_coordinates(metric),
        _base_nodes(metric, chart_lattice_for_frame(metric)),
    )


def _issues(record, parts, chains=()):
    vertices, faces, triangles = parts
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
        declared_straight_chains=tuple(chains),
    )


# --------------------------------------------------------------------------
# Размещение: коллинеарность по построению
# --------------------------------------------------------------------------


@pytest.mark.parametrize("shear", (0.5, 0.37, 1.3))
def test_a_free_chart_breaks_a_straight_generator_off_the_lattice_axes(shear):
    """Контроль предусловия: прежняя карта действительно теряла прямизну цепи."""

    parts = factories.slanted_cylinder(shear=shear)
    chain = factories.slanted_chain()
    chart = _chart(parts)
    assert not _on_one_line_in_order(_coordinates(chart.nodes, chain))
    record = build_metric(parts, declared_straight_chains=())
    with pytest.raises(SourceDeclaredStraightChainIsNotLinear):
        _queue_predicate(record, chain)


@pytest.mark.parametrize("shear", (0.5, 0.37, 1.3))
def test_a_declared_straight_chain_is_collinear_and_ordered_by_construction(shear):
    parts = factories.slanted_cylinder(shear=shear)
    chain = factories.slanted_chain()
    chart = _chart(parts, chain)
    assert _on_one_line_in_order(_coordinates(chart.nodes, chain))
    certificate = chart.certificate
    assert not stretch_violations(certificate.stretch)
    assert certificate.chart_boundary_overlap_count == 0
    assert (
        certificate.straight_chain_law
        is DevelopableStraightChainLawV1.INTERIOR_NODES_ON_ENDPOINT_SEGMENT_V1
    )
    assert [item.vertex_ids for item in certificate.declared_straight_chains] == [chain]
    # Всё, кроме внутренностей цепи, остаётся целым узлом решётки карты.
    interior = set(chain[1:-1])
    assert all(
        Fraction(node[axis]).denominator == 1
        for vertex, node in chart.nodes.items()
        if vertex not in interior
        for axis in range(2)
    )


@pytest.mark.parametrize("shear", (0.5, 0.37, 1.3))
def test_the_queue_predicate_accepts_the_placed_chain(shear):
    """Предикат очереди (`_chain_info`) принимает карту из публичного построителя."""

    parts = factories.slanted_cylinder(shear=shear)
    chain = factories.slanted_chain()
    record = build_metric(parts, declared_straight_chains=(chain,))
    assert type(record.metric.planarity_certificate) is DevelopableUnfoldCertificateV1
    info = _queue_predicate(record, chain)
    assert len(info.source) == len(chain)


def test_the_placement_moves_a_vertex_along_the_chord_by_less_than_an_eighth_of_a_node():
    """Сдвиг размещения — расстояние до хорды; вдоль хорды округление не больше восьмой узла."""

    parts = factories.slanted_cylinder(shear=0.5)
    chain = factories.slanted_chain()
    free = _chart(parts)
    placed = _chart(parts, chain)
    assert placed.chart_scale == free.chart_scale
    for vertex in chain[1:-1]:
        for axis in range(2):
            assert abs(Fraction(placed.nodes[vertex][axis]) - free.nodes[vertex][axis]) < 1
    others = [vertex for vertex in free.nodes if vertex not in set(chain[1:-1])]
    assert all(placed.nodes[vertex] == free.nodes[vertex] for vertex in others)


def test_a_chain_already_straight_on_the_lattice_keeps_its_nodes_bitwise():
    parts = factories.fold_grid()
    chain = tuple(SourceVertexId(f"v:g0_{j}") for j in range(5))
    free = _chart(parts)
    declared = _chart(parts, chain)
    assert _on_one_line_in_order(_coordinates(free.nodes, chain))
    assert declared.nodes == free.nodes
    assert declared.chart_scale == free.chart_scale
    assert declared.certificate.stretch == free.certificate.stretch
    assert declared.certificate.snap_residual == free.certificate.snap_residual
    assert declared.certificate.snapped_vertex_count == free.certificate.snapped_vertex_count
    assert [item.vertex_ids for item in declared.certificate.declared_straight_chains] == [chain]
    assert free.certificate.declared_straight_chains == ()


def test_a_chain_without_a_surface_is_ignored_and_a_chart_without_chains_is_unchanged():
    """Цепь, чьих вершин нет на карте, пропускается; без цепей карта прежняя."""

    parts = factories.slanted_cylinder()
    stranger = tuple(SourceVertexId(f"v:nowhere{j}") for j in range(3))
    assert _chart(parts, stranger).nodes == _chart(parts).nodes
    assert _chart(parts, stranger).certificate.declared_straight_chains == ()


# --------------------------------------------------------------------------
# Отказ: именованный, на ступени метрики, с доказательством
# --------------------------------------------------------------------------


def test_a_chain_the_chart_cannot_straighten_is_refused_by_a_metric_stage_name():
    parts = factories.kinked_fold_strip(0.25)
    chain = factories.kinked_chain()
    assert type(build_metric(parts).metric.planarity_certificate) is DevelopableUnfoldCertificateV1
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build_metric(parts, declared_straight_chains=(chain,))
    error = failure.value
    assert error.outcome is NamedOutcome.DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT
    message = str(error)
    assert "v:r0a..v:r3a" in message
    assert "worst vertex v:r" in message
    assert "side angle in [" in message and "rad against pi" in message
    assert "proven defect >=" in message
    assert "developable stretch" in message
    assert "[after near-planar NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in message


def test_the_bent_name_belongs_to_the_metric_stage_and_not_to_the_queue_declaration_error():
    """`SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR` — ошибка объявления хоста, не карты."""

    assert NamedOutcome.DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT.value.startswith("DEVELOPABLE_")
    assert "SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR" not in {
        item.value for item in NamedOutcome
    }


def test_a_chain_bent_within_the_budget_is_straightened_and_its_defect_is_recorded():
    """Прямизна по карману: цепь выпрямлена, а доказанный излом остаётся в сертификате."""

    parts = factories.kinked_fold_strip(0.0078125)
    chain = factories.kinked_chain()
    chart = _chart(parts, chain)
    assert _on_one_line_in_order(_coordinates(chart.nodes, chain))
    assert not stretch_violations(chart.certificate.stretch)
    (record,) = chart.certificate.declared_straight_chains
    assert record.defect_proven
    assert record.worst_vertex_id in chain[1:-1]
    low, high = Fraction(record.side_angle_enclosure.lower), Fraction(record.side_angle_enclosure.upper)
    assert low > Fraction(31415926, 10_000_000) and high < Fraction(32, 10)


def test_a_bent_chain_with_free_lines_is_not_blamed_on_the_stretch_of_other_triangles():
    """Без объявления та же поверхность разворачивается: имя отказа — прямизна, не растяжение."""

    parts = factories.kinked_fold_strip(0.25)
    chain = factories.kinked_chain()
    assert not stretch_violations(_chart(parts).certificate.stretch)
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _chart(parts, chain)
    assert failure.value.outcome is NamedOutcome.DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT


def test_a_surface_the_chart_refuses_anyway_keeps_its_own_refusal_name():
    """Карта, негодная и со свободными цепями, называет СВОЮ причину, а не прямизну."""

    parts = factories.spiral_strip()
    chain = tuple(SourceVertexId(f"v:r{k}a") for k in range(3))
    with pytest.raises(PlanarMetricAdmissionError) as plain:
        _chart(parts)
    with pytest.raises(PlanarMetricAdmissionError) as declared:
        _chart(parts, chain)
    assert declared.value.outcome is plain.value.outcome
    assert declared.value.outcome is not NamedOutcome.DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT


# --------------------------------------------------------------------------
# Планарные и near-planar байты не меняются
# --------------------------------------------------------------------------


def test_declared_chains_do_not_touch_planar_and_near_planar_bytes():
    from cftuv_envelope.codec import canonical_json_bytes

    flat = factories.surface(
        {
            "a": (0.0, 0.0, 0.0), "m": (1.0, 0.0, 0.0), "b": (2.0, 0.0, 0.0),
            "c": (2.0, 1.0, 0.0), "n": (1.0, 1.0, 0.0), "d": (0.0, 1.0, 0.0),
        },
        [["a", "m", "n", "d"], ["m", "b", "c", "n"]],
    )
    gentle = factories.bevel_strip(2, step_degrees=5.0)
    cases = (
        (flat, tuple(SourceVertexId(f"v:{name}") for name in "amb")),
        (gentle, tuple(SourceVertexId(f"v:r{k}a") for k in range(3))),
    )
    for parts, chain in cases:
        without = build_metric(parts)
        declared = build_metric(parts, declared_straight_chains=(chain,))
        assert canonical_json_bytes(declared) == canonical_json_bytes(without)
    assert type(build_metric(gentle).metric.planarity_certificate) is NearPlanarProjectionCertificateV1


# --------------------------------------------------------------------------
# Валидатор: пересчёт знает объявленные цепи, провод держит прямизну
# --------------------------------------------------------------------------


def test_the_validator_accepts_a_placed_chain_with_the_declared_chains_and_rejects_without():
    parts = factories.slanted_cylinder()
    chain = factories.slanted_chain()
    record = build_metric(parts, declared_straight_chains=(chain,))
    assert _issues(record, parts, (chain,)) == ()
    messages = [item.message for item in _issues(record, parts)]
    assert "developable certificate differs from exact recomputation" in messages


def test_the_validator_catches_a_chain_vertex_pushed_off_the_line():
    parts = factories.slanted_cylinder()
    chain = factories.slanted_chain()
    record = build_metric(parts, declared_straight_chains=(chain,))
    victim = chain[2]
    items = sorted(record.metric.exact_source_vertex_coordinates, key=lambda i: i.source_vertex_id.value)
    forged_items = []
    for item in items:
        if item.source_vertex_id == victim:
            point = item.domain_coordinate
            item = replace(
                item,
                domain_coordinate=ExactPoint2V1(
                    point.x, ExactRationalV1(point.y.numerator + point.y.denominator, point.y.denominator)
                ),
            )
        forged_items.append(item)
    forged = replace(
        record,
        metric=replace(record.metric, exact_source_vertex_coordinates=frozenset(forged_items)),
    )
    messages = [item.message for item in _issues(forged, parts, (chain,))]
    assert any("is not on one line of the chart in order" in message for message in messages)
    assert any("unfolded chart coordinates differ" in message for message in messages)


def test_the_validator_catches_a_forged_chain_record_and_a_forged_law():
    parts = factories.kinked_fold_strip(0.0078125)
    chain = factories.kinked_chain()
    record = build_metric(parts, declared_straight_chains=(chain,))
    certificate = record.metric.planarity_certificate
    (declared,) = certificate.declared_straight_chains
    hidden = replace(declared, worst_vertex_id=None, side_angle_enclosure=None, defect_proven=False)
    forged = replace(
        record,
        metric=replace(
            record.metric,
            planarity_certificate=replace(certificate, declared_straight_chains=(hidden,)),
        ),
    )
    assert "developable certificate differs from exact recomputation" in [
        item.message for item in _issues(forged, parts, (chain,))
    ]


def test_a_chain_record_names_distinct_vertices_and_an_interior_worst_vertex():
    from cftuv_envelope.contracts.metric import DevelopableDeclaredChainV1

    a, b, c = (SourceVertexId(f"v:{name}") for name in "abc")
    with pytest.raises(ValueError):
        DevelopableDeclaredChainV1((a, b), None, None, False)
    with pytest.raises(ValueError):
        DevelopableDeclaredChainV1((a, b, a), None, None, False)
    with pytest.raises(ValueError):
        DevelopableDeclaredChainV1((a, b, c), a, None, False)
    assert DevelopableDeclaredChainV1((a, b, c), None, None, False).defect_proven is False


def test_the_certificate_with_chains_roundtrips_through_its_codec():
    import cftuv_envelope as kernel

    record = build_metric(factories.kinked_fold_strip(0.0078125), declared_straight_chains=(factories.kinked_chain(),))
    codec = kernel.EmbeddingCertifiedRationalAffinePlanarMetricCodecV1
    assert codec.loads(codec.dumps(record)) == record


# --------------------------------------------------------------------------
# Единая выборка «объявленной прямой цепи» для хоста и валидатора
# --------------------------------------------------------------------------


def _vertex(name):
    return SourceVertexId(f"v:{name}")


def test_declared_straight_chains_are_open_chains_of_three_or_more_vertices_used_by_the_domain():
    def chain(name, vertices, closed=False):
        return SimpleNamespace(
            physical_chain_id=PhysicalChainId(name),
            ordered_source_vertex_ids=tuple(_vertex(item) for item in vertices),
            is_closed=closed,
        )

    chains = (
        chain("straight", "abcd"),
        chain("two", "ab"),
        chain("loop", "abcd", closed=True),
        chain("elsewhere", "efg"),
        chain("early", "abc"),
    )
    here, other = PatchDomainId("here"), PatchDomainId("other")
    uses = tuple(
        SimpleNamespace(physical_chain_id=PhysicalChainId(name), patch_domain_id=domain)
        for name, domain in (
            ("straight", here),
            ("two", here),
            ("loop", here),
            ("elsewhere", other),
            ("early", here),
        )
    )
    found = declared_straight_chain_vertices(chains, uses, here)
    assert found == (
        tuple(_vertex(item) for item in "abc"),
        tuple(_vertex(item) for item in "abcd"),
    )
    assert declared_straight_chain_vertices(chains, uses, other) == (
        tuple(_vertex(item) for item in "efg"),
    )
