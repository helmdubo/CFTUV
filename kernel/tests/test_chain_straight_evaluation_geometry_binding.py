from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
import cftuv_envelope.reference.compile as reference_compile
import cftuv_envelope.wavefront.conveyor as conveyor
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.evaluation_geometry import (
    ChainStraightBindingDisplacementExceeded,
    EvaluationGeometryBindingInvalid,
    EvaluationGeometryRefinementBudgetExhausted,
    _StraightChainInfo,
    _base_nodes,
    _chain_binding,
    _chain_info,
    _declared_straight_chains,
    _gram_matrix,
    _minimum_refinement_power,
    _refined_chain_bindings,
    _source_coordinates,
    chart_lattice_for_frame,
    evaluation_geometry_binding_residual,
    verify_evaluation_geometry_binding,
)


CASE_ROOT = (
    Path(__file__).parents[1]
    / "fixtures"
    / "sem_clb_02_lost_domains_v1"
    / "cases"
)


def _load_case(name: str):
    root = CASE_ROOT / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    frame = next(
        item
        for item in snapshot.surface_metric_descriptors
        if item.patch_domain_id == domain.patch_domain_id
    )
    return snapshot, request, domain, frame


def _compile_case(name: str):
    snapshot, request, domain, frame = _load_case(name)
    result = kernel.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is kernel.ReferenceOutcome.EXACT
    assert result.compilation is not None
    return snapshot, request, domain, frame, result.compilation


def _forged_v1(binding):
    return kernel.EvaluationGeometryBindingV1(
        schema_version=kernel.EVALUATION_GEOMETRY_BINDING_SCHEMA_V1,
        source_revision=binding.source_revision,
        patch_domain_id=binding.patch_domain_id,
        reference_metric_id=binding.reference_metric_id,
        binding_law=(
            kernel.EvaluationGeometryBindingLawV1.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1
        ),
        lattice_scale=binding.base_lattice_scale,
        source_vertex_coordinates=frozenset(
            kernel.EvaluationGeometrySourceVertexV1(
                item.source_vertex_id,
                item.base_bound_coordinate,
            )
            for item in binding.vertex_authorities
        ),
        bound_hidden_support_ids=binding.bound_hidden_support_ids,
    )


def test_v2_compiler_refines_until_no_clamp_exceeds_the_half_step():
    """Ёмкость хорды требовала r=1, но вершина зажималась на 0.176 м: берётся r=2 без зажима."""

    *_, compilation = _compile_case(
        "building_all_seams_patch_105_lost_resolved_v1"
    )
    binding = compilation.evaluation_geometry_binding

    assert type(binding) is kernel.ChainStraightEvaluationGeometryBindingV2
    assert binding.refinement_power == 2
    assert binding.lattice_scale == binding.base_lattice_scale * 4
    assert binding.previous_refinement_capacity_deficits == frozenset()
    (chain,) = binding.straight_chain_bindings
    assert (
        0,
        *(item.selected_k for item in chain.internal_assignments),
        chain.refined_endpoint_span_k,
    ) == (0, 3, 4)
    (assignment,) = chain.internal_assignments
    assert not assignment.clamped
    assert assignment.disposition is (
        kernel.ChainStraightAssignmentDispositionV2.UNCLAMPED_WITHIN_HALF_STEP
    )
    assert b"euclidean" not in (
        kernel.ChainStraightEvaluationGeometryBindingCodecV2.dumps(binding)
    ).lower()


def test_the_capacity_minimum_still_clamps_and_the_old_law_would_have_allowed_it():
    """Отрицательный контроль: при r=1 вершина 189 зажата на 0.1756 м (сверх полушага 0.125 м)."""

    snapshot, _request, _domain, frame, compilation = _compile_case(
        "building_all_seams_patch_105_lost_resolved_v1"
    )
    lattice = chart_lattice_for_frame(frame)
    source = _source_coordinates(frame)
    base = _base_nodes(frame, lattice)
    infos = tuple(
        _chain_info(chain, source, base)
        for chain in _declared_straight_chains(compilation)
    )
    gram = _gram_matrix(frame)

    assert _minimum_refinement_power(infos) == 1
    (clamped,) = _chain_binding(infos[0], lattice.scale, 1, gram).internal_assignments
    assert clamped.clamped
    assert clamped.disposition is (
        kernel.ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED
    )
    displacement = clamped.exact_gram_displacement_squared
    assert displacement.numerator / displacement.denominator == pytest.approx(
        0.1756**2, rel=1e-3
    )
    power, bindings = _refined_chain_bindings(infos, lattice.scale, gram)
    assert power == 2
    assert not any(
        item.clamped for chain in bindings for item in chain.internal_assignments
    )


def _crowded_chain_info():
    """Два внутренних вершины в одной миллиардной хорды: ни одно `r <= 8` их не разведёт."""

    ids = tuple(kernel.SourceVertexId(f"test:v{index}") for index in range(4))
    zero = Fraction(0)
    return _StraightChainInfo(
        chain=SimpleNamespace(
            physical_chain_id=kernel.PhysicalChainId("test:crowded-chain"),
            ordered_source_vertex_ids=ids,
        ),
        source=(
            (zero, zero),
            (Fraction(1, 2), zero),
            (Fraction(1, 2) + Fraction(1, 10**9), zero),
            (Fraction(1), zero),
        ),
        base_start_node=(0, 0),
        base_end_node=(1, 0),
        primitive_direction=(1, 0),
        base_endpoint_span=1,
    )


def test_a_clamp_no_refinement_can_remove_is_a_named_refusal_with_numbers():
    gram = ((Fraction(1), Fraction(0)), (Fraction(0), Fraction(1)))

    with pytest.raises(
        ChainStraightBindingDisplacementExceeded,
        match="CHAIN_STRAIGHT_BINDING_DISPLACEMENT_EXCEEDED",
    ) as refusal:
        _refined_chain_bindings((_crowded_chain_info(),), 1, gram)

    text = str(refusal.value)
    assert issubclass(
        ChainStraightBindingDisplacementExceeded, EvaluationGeometryBindingInvalid
    )
    assert "refinement ceiling r=8" in text
    assert "test:v2 on test:crowded-chain" in text
    assert "selected k=129 for unconstrained k=128 in [129, 255]" in text
    assert "Gram displacement" in text and "half-step bound" in text


def test_compile_maps_the_displacement_refusal_to_its_own_outcome(monkeypatch):
    snapshot, request, domain, _ = _load_case(
        "building_002_single_edge_patch_000_named_outcome_v1"
    )

    def _exceeded(*_args, **_kwargs):
        raise ChainStraightBindingDisplacementExceeded(
            "CHAIN_STRAIGHT_BINDING_DISPLACEMENT_EXCEEDED: numbers"
        )

    monkeypatch.setattr(
        reference_compile,
        "build_evaluation_geometry_binding",
        _exceeded,
    )
    result = reference_compile.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is (
        kernel.ReferenceOutcome.CHAIN_STRAIGHT_BINDING_DISPLACEMENT_EXCEEDED
    )
    assert result.compilation is None
    assert result.diagnostics[0].message.startswith(
        "CHAIN_STRAIGHT_BINDING_DISPLACEMENT_EXCEEDED"
    )


def test_reference_and_queue_use_recomputed_v2_coordinates_and_scale():
    snapshot, request, domain, frame, compilation = _compile_case(
        "building_all_seams_patch_105_lost_resolved_v1"
    )
    binding = compilation.evaluation_geometry_binding
    assert isinstance(
        binding,
        kernel.ChainStraightEvaluationGeometryBindingV2,
    )
    context = GeometryContext.build(compilation, frame)
    (chain,) = binding.straight_chain_bindings
    (assignment,) = chain.internal_assignments
    authority = next(
        item
        for item in binding.vertex_authorities
        if item.source_vertex_id == assignment.source_vertex_id
    )
    point = context.points_by_id[assignment.source_vertex_id]
    point_x, point_y = point.expressions()

    assert point_x == Fraction(
        authority.assigned_domain_coordinate.x.numerator,
        authority.assigned_domain_coordinate.x.denominator,
    )
    assert point_y == Fraction(
        authority.assigned_domain_coordinate.y.numerator,
        authority.assigned_domain_coordinate.y.denominator,
    )

    prepared = conveyor.prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    assert prepared.lattice is not None
    assert prepared.lattice.scale == binding.lattice_scale
    assert isinstance(
        prepared.compilation.evaluation_geometry_binding,
        kernel.ChainStraightEvaluationGeometryBindingV2,
    )


def test_v1_bytes_path_remains_selected_without_declared_multivertex_chain():
    snapshot, request, domain, _, compilation = _compile_case(
        "building_002_single_edge_patch_000_named_outcome_v1"
    )
    binding = compilation.evaluation_geometry_binding

    assert type(binding) is kernel.EvaluationGeometryBindingV1
    assert (
        kernel.EvaluationGeometryBindingCodecV1.loads(
            kernel.EvaluationGeometryBindingCodecV1.dumps(binding)
        )
        == binding
    )
    prepared = conveyor.prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    assert prepared.lattice is not None
    assert prepared.lattice.scale == binding.lattice_scale


def test_admitted_noncollinear_declaration_fails_with_frozen_named_outcome():
    snapshot, request, domain, _ = _load_case(
        "building_001_single_edge_patch_000_named_outcome_v1"
    )
    shell = SimpleNamespace(
        analysis_snapshot=snapshot,
        plan_key=SimpleNamespace(patch_domain_id=domain.patch_domain_id),
    )
    admitted = {
        item.physical_chain_id
        for item in _declared_straight_chains(shell)
    }
    assert kernel.PhysicalChainId(
        "host-v0:physical-chain:9d42cda796feb85637960858"
    ) in admitted

    result = kernel.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is (
        kernel.ReferenceOutcome.SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR
    )
    assert result.compilation is None
    assert result.diagnostics[0].message == (
        "SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR"
    )


def test_forged_v1_is_refused_by_reference_and_queue(monkeypatch):
    snapshot, request, domain, frame, compilation = _compile_case(
        "building_all_seams_patch_105_lost_resolved_v1"
    )
    forged = replace(
        compilation,
        evaluation_geometry_binding=_forged_v1(
            compilation.evaluation_geometry_binding
        ),
    )

    with pytest.raises(
        EvaluationGeometryBindingInvalid,
        match="requires V2 binding",
    ):
        verify_evaluation_geometry_binding(
            forged.evaluation_geometry_binding,
            forged,
            frame,
        )
    with pytest.raises(ReferenceGeometryError) as reference_error:
        GeometryContext.build(forged, frame)
    assert reference_error.value.outcome is (
        kernel.ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID
    )

    monkeypatch.setattr(
        conveyor,
        "compile_reference_envelopes",
        lambda *_args, **_kwargs: kernel.ReferenceCompileResultV1(
            kernel.ReferenceOutcome.EXACT,
            forged,
            (),
        ),
    )
    prepared = conveyor.prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    assert prepared.outcome is conveyor.ConveyorOutcome.DOMAIN_GEOMETRY_REFUSED
    assert prepared.detail == (
        kernel.ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID.value
    )


def test_refinement_overflow_is_named_and_compile_maps_the_outcome(monkeypatch):
    impossible = SimpleNamespace(
        base_endpoint_span=1,
        chain=SimpleNamespace(
            ordered_source_vertex_ids=tuple(range(258)),
        ),
    )
    with pytest.raises(
        EvaluationGeometryRefinementBudgetExhausted,
        match="REFINEMENT_BUDGET_EXHAUSTED",
    ):
        _minimum_refinement_power((impossible,))

    snapshot, request, domain, _ = _load_case(
        "building_002_single_edge_patch_000_named_outcome_v1"
    )

    def _exhausted(*_args, **_kwargs):
        raise EvaluationGeometryRefinementBudgetExhausted(
            "REFINEMENT_BUDGET_EXHAUSTED"
        )

    monkeypatch.setattr(
        reference_compile,
        "build_evaluation_geometry_binding",
        _exhausted,
    )
    result = reference_compile.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is kernel.ReferenceOutcome.REFINEMENT_BUDGET_EXHAUSTED
    assert result.compilation is None


def test_binding_residual_recomputes_over_refined_internal_movements():
    *_, frame, compilation = _compile_case(
        "building_all_seams_patch_105_lost_resolved_v1"
    )
    binding = compilation.evaluation_geometry_binding
    residual = evaluation_geometry_binding_residual(frame, binding)
    authority_residuals = tuple(
        abs(Fraction(component.numerator, component.denominator))
        for item in binding.vertex_authorities
        for component in (
            item.exact_offset_from_source.x,
            item.exact_offset_from_source.y,
        )
    )
    internal = next(
        item
        for item in binding.vertex_authorities
        if item.authority
        is kernel.ChainStraightVertexAuthorityV2.CHAIN_STRAIGHT_INTERNAL_REFINED_V2
    )

    assert residual == max(authority_residuals)
    assert any(
        component.numerator != 0
        for component in (
            internal.exact_offset_from_source.x,
            internal.exact_offset_from_source.y,
        )
    )
